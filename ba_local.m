function cost = ba_local(X0, model)
    load triple_models triple_models

    key_pts = triple_models{model, 4};
    
    % Split inputs
    M = X0(:, 6);
    S = X0(:, 7:end);
    
    % Cost 
    error = key_pts - M*S;
    cost = sum(sum(sqrt([error(1,:).^2 + error(2,:).^2; error(3,:).^2 + error(4,:).^2; error(5,:).^2 + error(6,:).^2])));
end