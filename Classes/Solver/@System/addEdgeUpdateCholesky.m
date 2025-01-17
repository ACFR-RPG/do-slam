function [obj] = addEdgeUpdateCholesky(obj,config,graph,edgeCell)

% addEdgeUpdate
iBlock = blockMap(obj,obj.iActiveEdges(edgeCell{2}),'edge');
residual = obj.b(iBlock,1);
[Omega, omega] = computeOmegas(config,graph,graph.edges(edgeCell{2}),residual);
iBlock = size(obj.L,1)+1:size(obj.L,1)+size(Omega,1);
if isempty(obj.L)
    H = (obj.covSqrtInv*obj.A)'*(obj.covSqrtInv*obj.A);
    L = chol(H,'lower');
    obj.L = L;
    c = (obj.covSqrtInv*obj.A)'*(obj.covSqrtInv*obj.b);
    obj.d = L\c;
end
% update L
L22 = obj.L(iBlock,iBlock) ;
L22_new=(chol(L22*L22'+Omega))';
obj.L(iBlock,iBlock)  = L22_new;
% update d
d2 = obj.d(iBlock,1);
obj.d(iBlock,1) = L22_new\(L22*d2 + omega);

end