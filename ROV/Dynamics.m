function  dxdt = Dynamics(t, state, input)  

    global M C D g J
    
    V = state(5:8);
    U = input;

    dxdt1 = J*V;
    dxdt2 = inv(M)*(U - (C+D)*V - g);
    dxdt  = [dxdt1; dxdt2];
    
end