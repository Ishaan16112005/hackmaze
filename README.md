# hackmaze
TO GET THE HISTOGRAM WITH THE VALUES RUN "python autoplot.py", THIS COMMAND CREATES TWO .jpg FILES THAT HAS THE HISTOGRAM OF THE LATENCY AND THE JITTER.
COMMANDS TO GET THE PROPER OUTPUT OF THE LEVEL 1 
iverilog -Wall -o sim.vvp *.v  (name anything of your choice as i have named sim in this case).
vvp sim.vvp
python3 autoplot.py 

