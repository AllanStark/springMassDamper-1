#!/bin/bash
#
# <bitbar.title>Bell Schedule</bitbar.title>
# <bitbar.version>v1.0</bitbar.version>
# <bitbar.author>Michael Klamkin</bitbar.author>
# <bitbar.author.github>klamike</bitbar.author.github>
# <bitbar.desc>Bell Schedule for Staten Island Technical High School.</bitbar.desc>
echo "| templateImage=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"
echo "---" #menubar icon
#echo "Bell Schedule | color=black"
#current time in unix time
hours="$(date +%H)"
hourssec=$(($hours * 3600))
hourssec=${hourssec#0}
mins="$(date +%M)"
minssec=$(($mins * 60))
minssec=${minssec#0}
sec="$(date +%S)"
sec=${sec#0}
sumsec=$(($hourssec + $minssec + $sec))
#period start and end values in unix time
prdStart1="28800"
prdEnd1="31260"
prdStart2="31500"
prdEnd2="33960"
prdStart3="34200"
prdEnd3="37020"
prdStart4="37260"
prdEnd4="39720"
prdStart5="39960"
prdEnd5="42420"
prdStart6="42660"
prdEnd6="45120"
prdStart7="45360"
prdEnd7="47820"
prdStart8="48060"
prdEnd8="50520"
prdStart9="50760"
prdEnd9="53220"
#prdStartt="76800"
#prdEndt="84780" #test times
##################figure out what period it is#############################################
#prd1
if [[ ! $(( sumsec )) < "$prdStart1" && ! $(( sumsec )) > "$prdEnd1" ]]; then
firstperiod="1"
tdiff=$((prdEnd1 - sumsec))
fi
#prd2
if [[ ! $(( sumsec )) < "$prdStart2" && ! $(( sumsec )) > "$prdEnd2" ]]; then
secondperiod="1"
tdiff=$((prdEnd2 - sumsec))
fi
#prd3
if [[ ! $(( sumsec )) < "$prdStart3" && ! $(( sumsec )) > "$prdEnd3" ]]; then
thirdperiod="1"
tdiff=$((prdEnd3 - sumsec))
fi
#prd4
if [[ ! $(( sumsec )) < "$prdStart4" && ! $(( sumsec )) > "$prdEnd4" ]]; then
fourthperiod="1"
tdiff=$((prdEnd4 - sumsec))
fi
#prd5
if [[ ! $(( sumsec )) < "$prdStart5" && ! $(( sumsec )) > "$prdEnd5" ]]; then
fifthperiod="1"
tdiff=$((prdEnd5 - sumsec))
fi
#prd6
if [[ ! $(( sumsec )) < "$prdStart6" && ! $(( sumsec )) > "$prdEnd6" ]]; then
sixthperiod="1"
tdiff=$((prdEnd6 - sumsec))
fi
#prd7
if [[ ! $(( sumsec )) < "$prdStart7" && ! $(( sumsec )) > "$prdEnd7" ]]; then
seventhperiod="1"
tdiff=$((prdEnd7 - sumsec))
fi
#prd8
if [[ ! $(( sumsec )) < "$prdStart8" && ! $(( sumsec )) > "$prdEnd8" ]]; then
eighthperiod="1"
tdiff=$((prdEnd8 - sumsec))
fi
#prd9
if [[ ! $(( sumsec )) < "$prdStart9" && ! $(( sumsec )) > "$prdEnd9" ]]; then
ninthperiod="1"
tdiff=$((prdEnd9 - sumsec))
fi
##prdt for testing
#if [[ ! $(( sumsec )) < "$prdStartt" && ! $(( sumsec )) > "$prdEndt" ]]; then
#tperiod="1"
#tdiff=$((prdEndt - sumsec))
#fi
# finding difference between current and end of period and echoing it at top of window
((sec=tdiff%60, tdiff/=60, min=tdiff%60, hrs=tdiff/60))
tdiffformat=$(printf "%d:%02d:%02d" $hrs $min $sec)
if [[ $hrs == "0" ]]; then 
tdiffformat=$(printf "%02d:%02d" $min $sec)
echo "Time Remaining: $tdiffformat | color = black"
fi
#echo "Time Remaining: $tdiffformat | color = black"

########################Highlighting the current period##########################################
if [[ $firstperiod == "1" ]]; then
echo "1                8:00       8:41 | color = black"
else
echo "1                8:00       8:41"
fi
if [[ $secondperiod == "1" ]]; then
echo "2               8:45       9:26 | color = black"
else
echo "2               8:45       9:26"
fi
if [[ $thirdperiod == "1" ]]; then
echo "3               9:30      10:17 | color = black"
else
echo "3               9:30      10:17"
fi
if [[ $fourthperiod == "1" ]]; then
echo "4              10:21      11:02 | color = black"
else
echo "4              10:21      11:02"
fi
if [[ $fifthperiod == "1" ]]; then
echo "5              11:06      11:47 | color = black"
else
echo "5              11:06      11:47"
fi
if [[ $sixthperiod == "1" ]]; then
echo "6              11:51      12:32 | color = black"
else
echo "6              11:51      12:32"
fi
if [[ $seventhperiod == "1" ]]; then
echo "7              12:36       1:17 | color = black"
else
echo "7              12:36       1:17"
fi
if [[ $eighthperiod == "1" ]]; then
echo "8                1:21       2:02 | color = black"
else
echo "8                1:21       2:02"
fi
if [[ $ninthperiod == "1" ]]; then
echo "9               2:06       2:47 | color = black"
else
echo "9               2:06       2:47"
fi
#if [[ $tperiod == "1" ]]; then  #for testing
#echo "t               d       m | color = black"
#else
#echo "t               d       m"
#fi