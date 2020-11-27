CAvallo=1

while [ 0 -lt $CAvallo ]
do

echo "------------------------------------------------"
echo "Selezionare direzione di movimento del veicolo"
echo "oppure goal da raggiungere"
echo "------------------------------------------------"

echo "             Interfaccia Comando"
echo "--------------------------------------"
echo "            || AVANTI     w ||"
echo "SINISTRA  a || STOP       s || DESTRA	d"
echo "            || INDIETRO   x ||"
echo "--------------------------------------"
echo "GOAL 10: 1 || GOAL 11: 2 || GOAL 20: 3 || GOAL 30: 4 ||GOAL 31: 5"
echo "--------------------------------------"

read data

case $data in
w)
   ( rostopic pub GOAL std_msgs/Int8 0  )&
   ( rostopic pub AUTO std_msgs/String w )&
   ;;
s)
   ( rostopic pub AUTO std_msgs/String s )&
   ;;
x)
   ( rostopic pub GOAL std_msgs/Int8 0  )&
   ( rostopic pub AUTO std_msgs/String x )&
   ;;
a)
   ( rostopic pub GOAL std_msgs/Int8 0  )&
   ( rostopic pub AUTO std_msgs/String a )&
   ;;
d)
   ( rostopic pub GOAL std_msgs/Int8 0  )&
   ( rostopic pub AUTO std_msgs/String d )&
   ;;
1)
   ( rostopic pub AUTO std_msgs/String k )&
   ( rostopic pub GOAL std_msgs/Int8 10  )&
   ;;
2)
   ( rostopic pub AUTO std_msgs/String k )&
   ( rostopic pub GOAL std_msgs/Int8 11  )&
   ;;
3)
   ( rostopic pub AUTO std_msgs/String k )&
   ( rostopic pub GOAL std_msgs/Int8 20  )&
   ;;
4)
   ( rostopic pub AUTO std_msgs/String k )&
   ( rostopic pub GOAL std_msgs/Int8 30  )&
   ;;
5)
   ( rostopic pub AUTO std_msgs/String k )&
   ( rostopic pub GOAL std_msgs/Int8 31  )&
   ;;
0)
   
esac 

$data=0;



done
