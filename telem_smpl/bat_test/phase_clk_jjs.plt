set key box
#set key graph 0.1, graph 0.1
#set key right top height 1.0 spacing 2
set key outside
set xlabel "4S Li-Po Bat voltage [V]"
set ylabel "telemetry code [dec]"
#set xtics 10.0
#set ytics 0.20
set grid
set xzeroaxis lw 0.5 lt -1
set size 1.5,1.5
set terminal postscript enhanced
#set terminal postscript eps
#set output "grayzone.eps"

set terminal png
set output "batv_code.png"

#set label 1 "{/Symbol m}=%1.3g [{/Symbol m}A], ", a, "  {/Symbol s}=%1.3g [{/Symbol m}A]", b at graph 0.65, graph 0.55
#set label 3 "grayzone=%1.3g [{/Symbol m}A]", gz at graph 0.65, graph 0.50

plot [][] 'bat_test_memo' using 1:2 w l title "Sg-X210-01 telem"
#,\
#'OUT' using 1:25 w l title "OUT10_JJ2"

set terminal windows
set output