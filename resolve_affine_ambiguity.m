function difference = resolve_affine_ambiguity(L)
    % load motion matrix 
    load M M
    
    % pre assign space for error
    difference = zeros(size(M, 1)/2, 4);
    
    %compute the residuals
    for i = 1:size(M,1)/2
        Ai = M(i*2-1:i*2,:);
        D = Ai*L*Ai' - eye(2);
        difference(i,:) = D(:);
    end
end