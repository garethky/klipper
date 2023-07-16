from klippy.extras.multiplex_adc import MultiplexAdcCaptureHelper
import sys, pytest

class MockPrinter:
    pass

class MockMcu:
    def seconds_to_clock(self, seconds):
        return seconds
    
    def register_response(self, callback, name, oid):
        pass

    def lookup_command(self, query):
        return "command"
    
    def lookup_query_command(self, query, query_end, oid):
        return "query_command"
    
    def clock_to_print_time(self, sclock):
        return 1.0

@pytest.mark.slow
def test_MultiplexAdcCaptureHelper_init():
    mcu = MockMcu()
    printer = MockPrinter()
    helper = MultiplexAdcCaptureHelper(printer, 1, mcu)
    assert helper != None

    sequence = 0x00
    for j in range(20000):
        for i in range(10):
            sequence += 0x01
            if sequence > 0xff:
                sequence = 0x00
            sample = {
                'sequence': sequence,
                'data': b'\x00X\xf6\xdc\xfe\x00\xfa\xa6\xdc\xfe\x00\x9bV\xdc\xfe'
            }
            helper._add_samples(sample)
        helper.flush()

if __name__ == '__main__':
    pytest.main(sys.argv)