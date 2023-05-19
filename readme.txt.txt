1. Flash the radar using uniFlash with the corresponding .bin file
2. Fill in ReadUart.py the right ComPorts numbers
3. Start running 


The ReadUART file is reading the time variation of the range profile vector for a person sitting in front of the radar at a distance of 0.5 meters.

In each second, the radar is emitting 20 frames (50ms is the frame duration),and since we are using a time window of 20 seconds, each saved file in recordings have 400 values.