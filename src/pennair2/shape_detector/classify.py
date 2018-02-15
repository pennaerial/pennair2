import sys
import stratify

# input: name of file, color to find, colors to ignore (default none)
if len(sys.argv) > 2:
	stratify.stratify(sys.argv[1], sys.argv[2], sys.argv[3:])
elif len(sys.argv) > 1:
	stratify.stratify(sys.argv[1], sys.argv[2])