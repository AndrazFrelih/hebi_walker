function [H] = Rot2(eul)
    H = RotX2(eul(1))*RotY2(eul(2))*RotZ2(eul(3));
end

