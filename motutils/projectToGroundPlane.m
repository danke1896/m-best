function [Xgp Ygp]=projectToGroundPlane(Xi, Yi, sceneInfo)
% project all points to ground plane

[F N]=size(Xi);
Xgp=zeros(size(Xi));
Ygp=zeros(size(Xi));

if length(sceneInfo.camPar)==1
    for t=1:F
        extar=find(Xi(t,:));
        for id=extar
            [Xgp(t,id) Ygp(t,id) zw]=imageToWorld(Xi(t,id), Yi(t,id), sceneInfo.camPar);
        end
    end
else
    for t=1:F
        extar=find(Xi(t,:));
        for id=extar
            [Xgp(t,id) Ygp(t,id) zw]=imageToWorld(Xi(t,id), Yi(t,id), sceneInfo.camPar(t));
        end
    end

end
end
