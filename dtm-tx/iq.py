import serial
import numpy as np

# s = serial.Serial('/dev/tty.usbmodem0004401673681')
s = serial.Serial('/dev/tty.usbmodem0004401665361')

s.flush()
done = False
fp = open('test.txt', 'a')
while True:
    msg = s.readline()
    print(msg)
    done = False
    if msg.startswith(b'status'):
        #  got first status
        count = 0
        while not done:
            parse = msg.split(b'len:')
            iq_length = int(parse[-1])
            print(parse[0])
            # print('len', iq_length)
            data = s.read(iq_length)
            data = np.frombuffer(data, dtype=np.uint8)
            data.tofile(fp, sep=',');
            fp.write('\n')
            print(count, data, len(data))
            count += 1
            msg = s.readline()
            # print(msg)
            msg = s.readline()
            if not msg.startswith(b'status'):
                done = True


