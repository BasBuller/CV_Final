function difference = resolve_affine_ambiguity(L)
    % load motion matrix 
    load triple_models triple_models
   
    % Set up complete M matrix
    n_im = max(size(triple_models));
    M = zeros(n_im*6, 3);
    for i = 1:n_im
        M((i-1)*6+1:i*6, :) = triple_models{i, 5};
    end
    
    % pre assign space for error
    difference = zeros(size(M, 1)/2, 4);
    
    %compute the residuals
    for i = 1:size(M,1)/2
        Ai = M(i*2-1:i*2,:);
        D = Ai*L*Ai' - eye(2);
        difference(i,:) = D(:);
    end
end