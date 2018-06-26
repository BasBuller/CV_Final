function cost = ba_local(X0, key_pts, model)
    % Split inputs
    if (model == 3)
        M = X0(:, 1:6)';
        S = X0(:, 7:end);
    else
        M = X0(:, 1:8)';
        S = X0(:, 9:end);
    end
        
    % Cost
    cost = 0;
    error = key_pts - M*S;
    for i = 1:model
        cost = cost + sum(sqrt(error((2*i-1), :).^2 + error(2*i, :).^2));
    end
end