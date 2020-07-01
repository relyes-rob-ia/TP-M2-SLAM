
function res = rmse(val1, val2)
    res = sqrt(mse(val1, val2));
end

function res = mse(val1, val2)
    diff = val1(:) - val2(:);
    res = sum(diff .^2) / size(val1(:), 1);
end

