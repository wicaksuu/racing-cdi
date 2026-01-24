#!/usr/bin/env python3
"""
Racing CDI - Web UI Bridge
Connects MCU serial to WebSocket for real-time web UI
"""

import asyncio
import json
import serial
import serial.tools.list_ports
from aiohttp import web
import aiohttp

# Global state
serial_port = None
serial_port_name = None
auto_connect = False  # Don't auto-connect, wait for user selection
clients = set()
current_data = {
    'rpm': 0,
    'timing': 0,
    'temp': 0,
    'battery': 0,
    'charging': 0,
    'map': 1,
    'limiter': 0,
    'flags': 0,
    'engineRunning': False,
    'overheating': False,
    'lowBattery': False,
    'killActive': False,
    'usingDefaultMap': False,
    'ignitionEnabled': True,
    'sdCardOk': True,
    'connected': False,
    'portName': '',
    'peak': 0,
    'cpu': 0,
    'ram': 0,
    'trigAngle': 0,
    'cut': 0,
    'engType': 2,
    'configSource': 0,  # 0=SD, 1=Flash, 2=Hardcoded
    'qsAdc': 0,         # Quick Shifter ADC raw value (0-4095)
    'tempRaw': 0,       # Temperature ADC raw (0-4095)
    'battRaw': 0,       # Battery ADC raw (0-4095)
    'chrgRaw': 0,       # Charging ADC raw (0-4095)
    'dRpm': 0,          # RPM change per cycle (+ = accel, - = decel)
    'phaseCorr': 0,     # Phase correction in ticks (±50 max)
    'predictiveMode': False,  # Predictive ignition active (timing > trigger angle)
    'timingClamped': False,   # Timing was clamped to min/max
    'skippedTriggers': 0      # Race condition skip counter (diagnostic)
}

def list_serial_ports():
    """List all available serial ports"""
    ports = serial.tools.list_ports.comports()
    result = []
    for port in ports:
        result.append({
            'device': port.device,
            'description': port.description,
            'hwid': port.hwid
        })
    return result

def parse_rt_data(line):
    """Parse RT:rpm,timing,temp,batt,charging,map,lim,flags,peak,cpu,ram,trig,cut,engType,cfgSrc,qsAdc,tempRaw,battRaw,chrgRaw,dRpm,phase,flags2,skip"""
    global current_data
    try:
        if line.startswith('RT:'):
            parts = line[3:].split(',')
            if len(parts) >= 11:
                current_data['rpm'] = int(parts[0])
                current_data['timing'] = int(parts[1])
                current_data['temp'] = int(parts[2])
                current_data['battery'] = int(parts[3])  # x10 (126 = 12.6V)
                current_data['charging'] = int(parts[4])  # x10
                current_data['map'] = int(parts[5])
                current_data['limiter'] = int(parts[6])
                flags = int(parts[7])
                current_data['flags'] = flags
                current_data['engineRunning'] = bool(flags & 0x01)
                current_data['overheating'] = bool(flags & 0x02)
                current_data['lowBattery'] = bool(flags & 0x04)
                current_data['killActive'] = bool(flags & 0x08)
                current_data['usingDefaultMap'] = bool(flags & 0x10)
                current_data['ignitionEnabled'] = bool(flags & 0x20)
                current_data['sdCardOk'] = bool(flags & 0x40)
                current_data['predictiveMode'] = bool(flags & 0x80)  # Predictive ignition
                current_data['peak'] = int(parts[8])
                current_data['cpu'] = int(parts[9])   # CPU usage %
                current_data['ram'] = int(parts[10])  # RAM usage %
                # Scope data (new fields)
                if len(parts) >= 14:
                    current_data['trigAngle'] = int(parts[11])  # Trigger angle BTDC
                    current_data['cut'] = int(parts[12])        # Last cycle cut (0/1)
                    current_data['engType'] = int(parts[13])    # Engine type (2/4)
                # Config source: 0=SD, 1=Flash, 2=Hardcoded
                if len(parts) >= 15:
                    current_data['configSource'] = int(parts[14])
                # Quick Shifter ADC value
                if len(parts) >= 16:
                    current_data['qsAdc'] = int(parts[15])
                # Raw ADC values for calibration
                if len(parts) >= 19:
                    current_data['tempRaw'] = int(parts[16])
                    current_data['battRaw'] = int(parts[17])
                    current_data['chrgRaw'] = int(parts[18])
                # Precision timing data (Expert Review optimizations)
                if len(parts) >= 21:
                    current_data['dRpm'] = int(parts[19])       # RPM change per cycle
                    current_data['phaseCorr'] = int(parts[20])  # Phase correction ticks
                # Extended flags and diagnostics (Expert Review fixes)
                if len(parts) >= 23:
                    flags2 = int(parts[21])
                    current_data['timingClamped'] = bool(flags2 & 0x01)
                    current_data['skippedTriggers'] = int(parts[22])
                return True
    except:
        pass
    return False

async def broadcast_data():
    """Broadcast current data to all WebSocket clients"""
    if clients:
        msg = json.dumps(current_data)
        await asyncio.gather(*[client.send_str(msg) for client in clients], return_exceptions=True)

async def broadcast_response(text):
    """Broadcast command response to all WebSocket clients"""
    if clients:
        msg = json.dumps({'type': 'response', 'data': text})
        await asyncio.gather(*[client.send_str(msg) for client in clients], return_exceptions=True)

async def connect_serial(port_name):
    """Connect to specified serial port"""
    global serial_port, serial_port_name, current_data

    # Disconnect existing connection first
    await disconnect_serial()

    try:
        serial_port = serial.Serial(port_name, 115200, timeout=0.1)
        serial_port_name = port_name
        current_data['connected'] = True
        current_data['portName'] = port_name
        await broadcast_data()
        return True
    except Exception as e:
        print(f"Failed to connect to {port_name}: {e}")
        return False

async def disconnect_serial():
    """Disconnect from current serial port"""
    global serial_port, serial_port_name, current_data

    if serial_port and serial_port.is_open:
        try:
            serial_port.close()
        except:
            pass

    serial_port = None
    serial_port_name = None
    current_data['connected'] = False
    current_data['portName'] = ''
    # Reset data when disconnected
    current_data['rpm'] = 0
    current_data['timing'] = 0
    current_data['engineRunning'] = False
    await broadcast_data()

async def serial_reader():
    """Read from serial and broadcast to WebSocket clients"""
    global serial_port, current_data

    while True:
        try:
            # Only read if connected (manual connection)
            if serial_port is None or not serial_port.is_open:
                current_data['connected'] = False
                current_data['portName'] = ''
                await asyncio.sleep(0.5)
                continue

            # Read line from serial
            if serial_port.in_waiting:
                line = serial_port.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    if parse_rt_data(line):
                        await broadcast_data()
                    else:
                        await broadcast_response(line)
            else:
                await asyncio.sleep(0.01)

        except serial.SerialException:
            serial_port = None
            current_data['connected'] = False
            current_data['portName'] = ''
            await broadcast_data()
            await asyncio.sleep(0.5)
        except:
            await asyncio.sleep(0.1)

async def websocket_handler(request):
    """Handle WebSocket connections"""
    ws = web.WebSocketResponse()
    await ws.prepare(request)

    clients.add(ws)

    # Send current state immediately (including port list)
    ports = list_serial_ports()
    init_data = {**current_data, 'ports': ports}
    await ws.send_str(json.dumps(init_data))

    try:
        async for msg in ws:
            if msg.type == aiohttp.WSMsgType.TEXT:
                try:
                    cmd = json.loads(msg.data)

                    # Handle port management commands
                    if 'action' in cmd:
                        if cmd['action'] == 'list_ports':
                            ports = list_serial_ports()
                            await ws.send_str(json.dumps({'type': 'ports', 'ports': ports}))

                        elif cmd['action'] == 'connect' and 'port' in cmd:
                            success = await connect_serial(cmd['port'])
                            await ws.send_str(json.dumps({
                                'type': 'connect_result',
                                'success': success,
                                'port': cmd['port']
                            }))

                        elif cmd['action'] == 'disconnect':
                            await disconnect_serial()
                            await ws.send_str(json.dumps({
                                'type': 'disconnect_result',
                                'success': True
                            }))

                    # Handle serial commands
                    elif 'command' in cmd and serial_port and serial_port.is_open:
                        serial_port.write((cmd['command'] + '\n').encode())

                except:
                    pass
    finally:
        clients.discard(ws)

    return ws

async def index_handler(request):
    """Serve the main HTML page"""
    return web.FileResponse('./index.html')

async def static_handler(request):
    """Serve static files"""
    filename = request.match_info.get('filename', 'index.html')
    return web.FileResponse(f'./{filename}')

async def start_background_tasks(app):
    app['serial_reader'] = asyncio.create_task(serial_reader())

async def cleanup_background_tasks(app):
    app['serial_reader'].cancel()
    try:
        await app['serial_reader']
    except asyncio.CancelledError:
        pass

def main():
    app = web.Application()
    app.router.add_get('/', index_handler)
    app.router.add_get('/ws', websocket_handler)
    app.router.add_get('/{filename}', static_handler)

    app.on_startup.append(start_background_tasks)
    app.on_cleanup.append(cleanup_background_tasks)

    print("Racing CDI - http://localhost:8080")

    web.run_app(app, host='0.0.0.0', port=8080, print=None)

if __name__ == '__main__':
    main()
