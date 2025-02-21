from pyModbusTCP.client import ModbusClient

def main():
    ip = "192.168.88.82"
    port = 6502

    client = ModbusClient(host=ip, port=port, auto_open=False)

    print("type(client.open) =", type(client.open))  # Should be <class 'method'>
    connected = client.open()
    print("connected =", connected)
    if not connected:
        print("Could not connect.")
    else:
        print("Connected OK!")

if __name__ == "__main__":
    main()