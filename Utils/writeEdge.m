%--------------------------------------------------------------------------
% Author: Montiel Abello - montiel.abello@gmail.com - 23/05/17
% Contributors:
%--------------------------------------------------------------------------

function writeEdge(label,vIn,vOut,value,covariance,fileID)
%WRITEEDGE writes edge to graph file

cov_dim = size(covariance);
if cov_dim(1) == 6
    tmp = covariance(1:3, 1:3);
    covariance(1:3, 1:3) = covariance(4:6, 4:6);
    covariance(4:6, 4:6) = tmp;
end

covariance = covToUpperTriVec(covariance);
formatSpec = strcat('%s',...
                    repmat(' %d',1,numel(vIn)),...
                    repmat(' %d',1,numel(vOut)),...
                    repmat(' %0.9f',1,numel(value)),...
                    repmat(' %0.9f',1,numel(covariance)),'\n');
cov_inv = covariance;
cov_inv(cov_inv~=0) = 1./cov_inv(cov_inv~=0);
fprintf(fileID,formatSpec,label,vIn,vOut,value,cov_inv);
end

