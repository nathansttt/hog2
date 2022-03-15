#!/bin/bash

for arg in arena2 brc501d den005d den204d den408d den900d lak100n lak200d lak400d lak511d lgt602d orz107d orz700d ost001d rmtst03 arena brc502d den009d den206d den500d den901d lak101d lak201d lak401d lak512d lgt603d orz200d orz701d ost002d rmtst brc000d brc503d den011d den207d den501d den998d lak102d lak202d lak403d lak513d lgt604d orz201d orz702d ost003d brc100d brc504d den012d den308d den502d hrt000d lak103d lak203d lak404d lak514d lgt605d orz203d orz703d ost004d brc101d brc505d den020d den312d den504d hrt001d lak104d lak250d lak405d lak515d orz000d orz300d orz704d ost100d brc200d brc997d den101d den400d den505d hrt002d lak105d lak300d lak503d lak519d orz100d orz301d orz800d ost101d brc201d brc999d den200d den401d den510d hrt201d lak106d lak302d lak504d lak526d orz101d orz302d orz900d ost102d brc202d combat2 den200n den403d den520d hrt201n lak107d lak303d lak505d lgt101d orz102d orz303d orz901d oth000d brc203d combat den201d den404d den600d isound1 lak108d lak304d lak506d lgt300d orz103d orz304d orz999d oth001d brc204d den000d den202d den405d den601d lak100c lak109d lak307d lak507d lgt600d orz105d orz500d ost000a oth999d brc300d den001d den203d den407d den602d lak100d lak110d lak308d lak510d lgt601d orz106d orz601d ost000t rmtst01

do
    ~/hog2/bin/release/BGS -inconsistent ~/hog2/maps/dao/$arg.map ~/hog2/scenarios/dao/$arg.map.scen astar | grep ^result | awk '{ sum += $3; n++ } END {print sum/n; print n}'
    ~/hog2/bin/release/BGS -inconsistent ~/hog2/maps/dao/$arg.map ~/hog2/scenarios/dao/$arg.map.scen bpmx | grep ^result | awk '{ sum += $3; n++ } END {print sum/n; print n}'
    ~/hog2/bin/release/BGS -inconsistent ~/hog2/maps/dao/$arg.map ~/hog2/scenarios/dao/$arg.map.scen IBEX | grep ^result | awk '{ sum += $3; n++ } END {print sum/n; print n}'
    ~/hog2/bin/release/BGS -inconsistent ~/hog2/maps/dao/$arg.map ~/hog2/scenarios/dao/$arg.map.scen bgs-bpmx-1 | grep ^result | awk '{ sum += $3; n++ } END {print sum/n; print n}'
    ~/hog2/bin/release/BGS -inconsistent ~/hog2/maps/dao/$arg.map ~/hog2/scenarios/dao/$arg.map.scen bgs-1 | grep ^result | awk '{ sum += $3; n++ } END {print sum/n; print n}'
    ~/hog2/bin/release/BGS -inconsistent ~/hog2/maps/dao/$arg.map ~/hog2/scenarios/dao/$arg.map.scen bgs-bpmx-2 | grep ^result | awk '{ sum += $3; n++ } END {print sum/n; print n}'
    ~/hog2/bin/release/BGS -inconsistent ~/hog2/maps/dao/$arg.map ~/hog2/scenarios/dao/$arg.map.scen bgs-2 | grep ^result | awk '{ sum += $3; n++ } END {print sum/n; print n}'
    ~/hog2/bin/release/BGS -inconsistent ~/hog2/maps/dao/$arg.map ~/hog2/scenarios/dao/$arg.map.scen bgs-bpmx-3 | grep ^result | awk '{ sum += $3; n++ } END {print sum/n; print n}'
    ~/hog2/bin/release/BGS -inconsistent ~/hog2/maps/dao/$arg.map ~/hog2/scenarios/dao/$arg.map.scen bgs-3 | grep ^result | awk '{ sum += $3; n++ } END {print sum/n; print n}'
    ~/hog2/bin/release/BGS -inconsistent ~/hog2/maps/dao/$arg.map ~/hog2/scenarios/dao/$arg.map.scen bgs-bpmx-4 | grep ^result | awk '{ sum += $3; n++ } END {print sum/n; print n}'
    ~/hog2/bin/release/BGS -inconsistent ~/hog2/maps/dao/$arg.map ~/hog2/scenarios/dao/$arg.map.scen bgs-4 | grep ^result | awk '{ sum += $3; n++ } END {print sum/n; print n}'
    ~/hog2/bin/release/BGS -inconsistent ~/hog2/maps/dao/$arg.map ~/hog2/scenarios/dao/$arg.map.scen bgs-bpmx-5 | grep ^result | awk '{ sum += $3; n++ } END {print sum/n; print n}'
    ~/hog2/bin/release/BGS -inconsistent ~/hog2/maps/dao/$arg.map ~/hog2/scenarios/dao/$arg.map.scen bgs-5 | grep ^result | awk '{ sum += $3; n++ } END {print sum/n; print n}'


    ~/hog2/bin/release/BGS -inconsistent ~/hog2/maps/dao/$arg.map ~/hog2/scenarios/dao/$arg.map.scen astar | grep ^result | awk '{ sum += $2; n++ } END {print sum/n; print n}'
    ~/hog2/bin/release/BGS -inconsistent ~/hog2/maps/dao/$arg.map ~/hog2/scenarios/dao/$arg.map.scen bpmx | grep ^result | awk '{ sum += $2; n++ } END {print sum/n; print n}'
    ~/hog2/bin/release/BGS -inconsistent ~/hog2/maps/dao/$arg.map ~/hog2/scenarios/dao/$arg.map.scen IBEX | grep ^result | awk '{ sum += $2; n++ } END {print sum/n; print n}'
    ~/hog2/bin/release/BGS -inconsistent ~/hog2/maps/dao/$arg.map ~/hog2/scenarios/dao/$arg.map.scen bgs-bpmx-1 | grep ^result | awk '{ sum += $2; n++ } END {print sum/n; print n}'
    ~/hog2/bin/release/BGS -inconsistent ~/hog2/maps/dao/$arg.map ~/hog2/scenarios/dao/$arg.map.scen bgs-1 | grep ^result | awk '{ sum += $2; n++ } END {print sum/n; print n}'
    ~/hog2/bin/release/BGS -inconsistent ~/hog2/maps/dao/$arg.map ~/hog2/scenarios/dao/$arg.map.scen bgs-bpmx-2 | grep ^result | awk '{ sum += $2; n++ } END {print sum/n; print n}'
    ~/hog2/bin/release/BGS -inconsistent ~/hog2/maps/dao/$arg.map ~/hog2/scenarios/dao/$arg.map.scen bgs-2 | grep ^result | awk '{ sum += $2; n++ } END {print sum/n; print n}'
    ~/hog2/bin/release/BGS -inconsistent ~/hog2/maps/dao/$arg.map ~/hog2/scenarios/dao/$arg.map.scen bgs-bpmx-3 | grep ^result | awk '{ sum += $2; n++ } END {print sum/n; print n}'
    ~/hog2/bin/release/BGS -inconsistent ~/hog2/maps/dao/$arg.map ~/hog2/scenarios/dao/$arg.map.scen bgs-3 | grep ^result | awk '{ sum += $2; n++ } END {print sum/n; print n}'
    ~/hog2/bin/release/BGS -inconsistent ~/hog2/maps/dao/$arg.map ~/hog2/scenarios/dao/$arg.map.scen bgs-bpmx-4 | grep ^result | awk '{ sum += $2; n++ } END {print sum/n; print n}'
    ~/hog2/bin/release/BGS -inconsistent ~/hog2/maps/dao/$arg.map ~/hog2/scenarios/dao/$arg.map.scen bgs-4 | grep ^result | awk '{ sum += $2; n++ } END {print sum/n; print n}'
    ~/hog2/bin/release/BGS -inconsistent ~/hog2/maps/dao/$arg.map ~/hog2/scenarios/dao/$arg.map.scen bgs-bpmx-5 | grep ^result | awk '{ sum += $2; n++ } END {print sum/n; print n}'
    ~/hog2/bin/release/BGS -inconsistent ~/hog2/maps/dao/$arg.map ~/hog2/scenarios/dao/$arg.map.scen bgs-5 | grep ^result | awk '{ sum += $2; n++ } END {print sum/n; print n}'

done