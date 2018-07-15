#!/bin/sh
VARS="A0 A1 B0 B1"

for defname in $VARS; do
	cat <<EOS
#ifdef ${defname}
# define NEED_${defname}_RESTORED ${defname}
# undef ${defname}
#endif
EOS
done

echo -----------------------------------

for defname in $VARS; do
	cat <<EOS
#ifdef NEED_${defname}_RESTORED
# define ${defname} NEED_${defname}_RESTORED
# undef NEED_${defname}_RESTORED
#endif
EOS
done
