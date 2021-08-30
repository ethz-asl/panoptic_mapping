import os
import shutil
import sys

# Arguments are the source directory and the target directory
if len(sys.argv) != 4:
    print("Arguments need to be 'source directory' 'target directory "
          "number of scenes'. Exiting.")
    sys.exit()
src = sys.argv[1]
dst = sys.argv[2]
num = int(sys.argv[3])

folders = sorted([
    d for d in os.listdir(src) if os.path.isdir(os.path.join(sys.argv[1], d))
],
                 reverse=True)

if len(folders) < num:
    print("Found fewer than %i entries in '%s'. Exiting." % (num, src))
    sys.exit()

for i in range(num):
    source = os.path.join(src, folders[i])
    target = os.path.join(dst, "pan_%i" % (num - i - 1))
    shutil.move(source, target)
    print("Moved '%s' to '%s'." % (source, target))

print("Successfully finished moving logs.")
