function dxdt = Dynamic_model(t,State)
    global M C D g U J
    V = State(7:12);
    dxdt1 = J*V;
    dxdt2 = inv(M)*(U  - C*V - D*V - g); 
    dxdt = [dxdt1; dxdt2];
end