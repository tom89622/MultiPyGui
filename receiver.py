import serial

port = serial.Serial('/dev/ttys048', 115200)  # Adjust to the other virtual port

MSP_HEADER = "$M<"

def decode_msp(byte_array):
    """Decode an MSP message from a received byte array."""
    
    # Ensure we have at least enough bytes for header, size, command, and checksum
    if len(byte_array) < 6:
        raise ValueError("Byte array too short to be a valid MSP message")

    # Check for correct header
    if byte_array[:3] != b'$M<':
        raise ValueError("Invalid MSP header")

    # Extract payload size
    payload_size = byte_array[3]
    
    # Extract command identifier
    command = byte_array[4]

    # Extract payload (if any)
    payload = byte_array[5:5 + payload_size]

    # Extract checksum
    received_checksum = byte_array[5 + payload_size]

    # Calculate checksum (XOR of payload size, command, and each byte in the payload)
    calculated_checksum = payload_size ^ command
    for byte in payload:
        calculated_checksum ^= byte

    # Verify checksum
    if calculated_checksum != received_checksum:
        raise ValueError("Checksum mismatch")

    return {
        "command": command,
        "payload": list(payload),
        "checksum_valid": True
    }

while True:
    data = port.readline()
    if data:
        # Print raw received data in hex format
        decoded_message = decode_msp(data)
        print("Received MSP data (hex):", ' '.join(f'{byte:02X}' for byte in data))
        print(f"Decoded MSP Message: {decoded_message}")
