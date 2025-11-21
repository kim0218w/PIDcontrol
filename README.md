# PIDcontrol
모터제어에서 PID제어를 한 코드내용


python3 << 'EOF'
import smbus
import time

bus = smbus.SMBus(1)
MUX_ADDR = 0x70
ENCODER_ADDR = 0x36

print("Multiplexer detected at 0x70")
print("\nTrying to find AS5600 on multiplexer channels...\n")

# 모든 채널 (0-7) 테스트
for channel in range(8):
    try:
        # 채널 활성화
        bus.write_byte(MUX_ADDR, 1 << channel)
        time.sleep(0.01)
        
        # AS5600 읽기 시도
        data = bus.read_byte_data(ENCODER_ADDR, 0x0B)
        print(f"✓ Channel {channel}: AS5600 found! Status = 0x{data:02X}")
        
        # 각도 읽기 테스트
        angle_data = bus.read_i2c_block_data(ENCODER_ADDR, 0x0C, 2)
        raw = ((angle_data[0] << 8) | angle_data[1]) & 0x0FFF
        angle = raw * 360.0 / 4096.0
        print(f"  Current angle: {angle:.2f} degrees")
        
    except Exception as e:
        print(f"✗ Channel {channel}: No AS5600")

# Multiplexer 비활성화
bus.write_byte(MUX_ADDR, 0x00)
print("\nDone!")
EOF
