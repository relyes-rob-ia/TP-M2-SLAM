function [z_list, h_list, r_list] = prepare(N, Z, Hfull, Rv)
    z_list = cell(N, 1);
    h_list = cell(N, 1);
    r_list = cell(N, 1);
    
    for k = 1:N
        if isnan(Z(1,k)) && isnan(Z(3,k))
            z_list(k) = {NaN};
            h_list(k) = {NaN};
            r_list(k) = {NaN};
        elseif isnan(Z(1,k)) 
            z_list(k) = {Z(3:4, k)};
            h_list(k) = {Hfull(3:4, :)};
            r_list(k) = {Rv(3:4, 3:4)};
        elseif isnan(Z(3,k))
            z_list(k) = {Z(1:2, k)};
            h_list(k) = {Hfull(1:2, :)};
            r_list(k) = {Rv(1:2, 1:2)};
        else
            z_list(k) = {Z};
            h_list(k) = {Hfull};
            r_list(k) = {Rv};
        end
    end
end
