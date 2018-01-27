import sys
import stratify

# input: name of file
# assumption: all shapes are yellow
# todo: fix the above assumption limitation
if len(sys.argv) > 1:
	stratify.stratify(sys.argv[1], sys.argv[2])