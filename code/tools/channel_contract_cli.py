import argparse
import sys

from nexus_channel_contract import decode_envelope, encode_envelope


def main(argv=None):
    parser = argparse.ArgumentParser(description="Validate a NEXUS channel envelope")
    parser.add_argument("--input", required=True, help="JSON envelope to validate")
    parser.add_argument("--output", help="optional normalized JSON output")
    args = parser.parse_args(argv)
    with open(args.input, "rb") as stream:
        encoded = encode_envelope(decode_envelope(stream.read()))
    if args.output:
        with open(args.output, "wb") as stream:
            stream.write(encoded + b"\n")
    else:
        sys.stdout.buffer.write(encoded + b"\n")


if __name__ == "__main__":
    main()
