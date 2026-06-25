### import subfiles ###
import asyncio
import socket
import time

data_queue = asyncio.Queue()

async def wifi_connect(host: str, port: int, timeout: float = 10.0):
    reader, writer = None, None
    try:
        # Wrap the connection attempt with an explicit timeout
        reader, writer = await asyncio.wait_for(
            asyncio.open_connection(host, port), 
            timeout=timeout
        )
        print(f"Successfully connected to {host}:{port}")
        return reader, writer

    except TimeoutError:
        print(f"Connection error: Attempt to connect to {host}:{port} timed out.")
    except ConnectionRefusedError:
        print(f"Connection error: Server at {host}:{port} refused the connection.")
    except socket.gaierror:
        print(f"Connection error: Could not resolve hostname '{host}'.")
    except OSError as e:
        print(f"Network error occurred: {e}")
    except asyncio.CancelledError:
        print("Connection task was cancelled.")
        raise  # It is best practice to re-raise CancelledError in asyncio
    except Exception as e:
        print(f"Unexpected error: {e}")
        
    # Ensure partial connections are closed if an error occurs post-creation
    if writer:
        writer.close()
        await writer.wait_closed()
        
    return None, None

async def wifi_read(reader):
    try:
        while True:
            data = await reader.read(1024)
            if not data:
                print("Connection closed by server.")
                break
            print(f"Received: {data.decode().strip()}")
            return data.decode().strip()
    except asyncio.CancelledError:
        pass

async def wifi_write(writer, message):
    try:
        message = f"{message}"
        writer.write(message.encode())
        await writer.drain()
        print(f"Sent: {message}")
#         await asyncio.sleep(1)
    except asyncio.CancelledError:
        pass

async def wifi_disconnect(s):
    writer.close()
    await writer.wait_closed()
