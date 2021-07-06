function Show_Simulation(eta_trajectory)
    count = length(eta_trajectory);
    for eta = eta_trajectory
        count = count - 1;
        eta_cell = eta{1};
        eta_list = eta_cell{1};
        X = eta_list(1:6);
        V = eta_list(7:12); 

        x = X(1);
        y = X(2);
        z = X(3);
        ph = X(4);
        th = X(5);
        ps = X(6);

        p1 = [-2; +3;  1];
        p2 = [ 2; +3;  1];
        p3 = [ 2; -3;  1];
        p4 = [-2; -3;  1];

        p5 = [-2; +3; -1];
        p6 = [ 2; +3; -1];
        p7 = [ 2; -3; -1];
        p8 = [-2; -3; -1];

%         Rotation_matrix = [                          cos(th)*cos(ps),                           -cos(ph)*sin(ps),          sin(th);
%                            sin(ph)*sin(th)*cos(ps) + cos(ph)*sin(ps), -sin(ph)*sin(th)*sin(ps) + cos(ph)*cos(ps), -sin(ph)*cos(th);
%                           -cos(ph)*sin(th)*cos(ps) + sin(ph)*sin(ps),  cos(ph)*sin(th)*sin(ps) + sin(ph)*cos(ps),  cos(ps)*cos(th)];
                      
        Rotation_matrix = [ cos(th)*cos(ps),  sin(ph)*sin(th)*cos(ps) + cos(ph)*sin(ps), -cos(ph)*sin(th)*cos(ps) + sin(ph)*sin(ps);
                           -cos(th)*sin(ps), -sin(ph)*sin(th)*sin(ps) + cos(ph)*cos(ps),  cos(ph)*sin(th)*sin(ps) + sin(ph)*cos(ps);
                                    sin(th),                           -sin(ph)*cos(th),                            cos(ph)*cos(th)];
                                
        p1 = Rotation_matrix*p1 + [x;y;z];
        p2 = Rotation_matrix*p2 + [x;y;z];
        p3 = Rotation_matrix*p3 + [x;y;z];
        p4 = Rotation_matrix*p4 + [x;y;z];
        p5 = Rotation_matrix*p5 + [x;y;z];
        p6 = Rotation_matrix*p6 + [x;y;z];
        p7 = Rotation_matrix*p7 + [x;y;z];
        p8 = Rotation_matrix*p8 + [x;y;z];
        
        subplot(1,2,1)
        plot3(x,y,z,'.','Markersize',10,'color','k');
        title('BlueRov2 LQR Control Simulation(3D)');
        drawBox(p1,p2,p3,p4,p5,p6,p7,p8);
        xlim([-10 10])
        ylim([-10 10])
        zlim([-20 5])
        xlabel('X') 
        ylabel('Y')
        zlabel('Z')
        grid on
        
        subplot(1,2,2)
        plot(x,y,'.','Markersize',10,'color','k')
        title('BlueRov2 LQR Control Simulation(2D)');
        line([p1(1) p2(1) p3(1) p4(1) p1(1)],[p1(2) p2(2) p3(2) p4(2) p1(2)],[p1(3)-1 p2(3)-1 p3(3)-1 p4(3)-1 p1(3)-1]);
        xlim([-10 10])
        ylim([-10 10])
        xlabel('X') 
        ylabel('Y')
        grid on
        
        if count == 1
            hold on
        else
            hold off
        end
        
%         subplot(2,2,3)
%         plot3(x,y,z,'.','Markersize',5,'color','k');
%         title('Path(3D)');
%         xlim([-10 10])
%         ylim([-10 10])
%         zlim([-20 5])
%         xlabel('X') 
%         ylabel('Y')
%         zlabel('Z')
%         grid on
%         hold on
%         
%         subplot(2,2,4)
%         plot(x,y,'.','Markersize',5,'color','k');
%         title('Path(2D)');
%         xlim([-10 10])
%         ylim([-10 10])
%         xlabel('X') 
%         ylabel('Y')
%         grid on
%         hold on
        drawnow
    end
end