#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# import sys
# from optparse import OptionParser

from dynamixel_driver import dynamixel_io

if __name__ == '__main__':
    """
    parser = OptionParser(usage='Usage: %prog [options]', description='Changes the unique ID of a Dynamixel servo motor.')
    parser.add_option('-p', '--port', metavar='PORT', default='/dev/ttyUSB0',
                      help='motors of specified controllers are connected to PORT [default: %default]')
    parser.add_option('-b', '--baud', metavar='BAUD', type="int", default=1000000,
                      help='connection to serial port will be established at BAUD bps [default: %default]')
    parser.add_option('-f', '--from-id', metavar='FROM_ID', type="int", default=1,
                      help='from id [default: %default]')
    parser.add_option('-t', '--to-id', metavar='TO_ID', type="int", default=7,
                      help='to id [default: %default]')
                      
    (options, args) = parser.parse_args(sys.argv)
    
    if len(args) < 1:
        parser.print_help()
        exit(1)
    """

    from_id = 1 
    to_id = 7 
    port = "/dev/ttyUSB0"
    baudrate = 1000000
    
    try:
        dxl_io = dynamixel_io.DynamixelIO(port, baudrate)
    except dynamixel_io.SerialOpenError as soe:
        print('ERROR:', soe)
    else:
        for idx in [x + from_id for x in range(to_id - from_id + 1)]:
            print('Scanning %d...' %(idx), end=' ')
            if dxl_io.ping(idx):
                print('[SUCCESS] : id %d respond to a ping' %(idx))
            else:
                print('[ERROR] : id %d not respond.' % idx)
