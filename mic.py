import socket
import pyaudio
import time

# 设置音频参数（必须和ESP32完全匹配）
CHUNK = 1024  # 每个数据包的大小
FORMAT = pyaudio.paInt16  # 16位整型
CHANNELS = 1  # 单声道
RATE = 16000  # 采样率16kHz
BUFFER_SIZE = CHUNK * 4  # 增加播放缓冲区，避免断音

# 创建PyAudio对象
p = pyaudio.PyAudio()

# 打开音频流（增加缓冲区，避免播放卡顿）
stream = p.open(format=FORMAT,
                channels=CHANNELS,
                rate=RATE,
                output=True,
                frames_per_buffer=CHUNK,
                output_device_index=None)  # 如果没声音，可指定声卡索引

# 设置UDP服务器
server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, BUFFER_SIZE * 2)
server_socket.bind(('192.168.10.120', 8888))
server_socket.setblocking(False)  # 非阻塞模式，避免卡死

print(f"UDP音频服务器启动，监听端口8888...")
print(f"音频参数：{RATE}Hz, {CHANNELS}声道, 16位")

audio_buffer = b''  # 音频缓冲，避免断音
last_time = time.time()

try:
    while True:
        try:
            # 接收数据（缓冲区设为CHUNK*2，对应int16的字节数）
            data, addr = server_socket.recvfrom(CHUNK * 2)
            if data:
                audio_buffer += data
                print(f"从{addr}接收{len(data)}字节数据，缓冲总长度：{len(audio_buffer)}", end='\r')
                
                # 当缓冲区足够时播放
                if len(audio_buffer) >= CHUNK * 2:
                    stream.write(audio_buffer[:CHUNK * 2])
                    audio_buffer = audio_buffer[CHUNK * 2:]
                    
                last_time = time.time()
                
        except BlockingIOError:
            # 没有数据时的空转
            if time.time() - last_time > 5:
                print("\n5秒未收到数据，请检查ESP32连接！")
            time.sleep(0.001)
            continue
            
        except Exception as e:
            print(f"\n错误：{e}")
            break
            
except KeyboardInterrupt:
    print("\n用户终止程序")
    
finally:
    # 清理资源
    stream.stop_stream()
    stream.close()
    p.terminate()
    server_socket.close()
    print("资源已释放，程序退出")