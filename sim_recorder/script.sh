#!/bin/bash
cd ram
for i in {0..74}; do 
   mv "ram_$i.csv" "ram_$(($i+76)).csv";
done
cd ../cpu 
for i in {0..74}; do 
   mv "cpu_$i.csv" "cpu_$(($i+76)).csv";
done

