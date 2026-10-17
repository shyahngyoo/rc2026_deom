"""Byte-for-byte C interoperability vectors; independent of serial hardware."""
import json
from pathlib import Path
from quadruped_stm32_bridge.protocol import encode_frame, FrameParser


def test_published_c_vectors_match_codec_and_stream_decoder():
    document=json.loads((Path(__file__).resolve().parents[1]/'protocol/test_vectors.json').read_text())
    for v in document['vectors']:
        packet=bytes.fromhex(v['hex'])
        assert encode_frame(v['kind'],v['sequence'],v['session'],v['payload'])==packet
        frames=FrameParser().feed(packet)
        assert len(frames)==1 and frames[0].payload==v['payload']
        assert frames[0].session==v['session'] and frames[0].sequence==v['sequence']
