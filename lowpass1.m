function res = lowpass3(input,input_old,output_old,T,cutoff)
    tau = 1/(2*pi*cutoff);
    res = (T*(input+input_old)-(T-2*tau)*output_old)/(2*tau+T);

end