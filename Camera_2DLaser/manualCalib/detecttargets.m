%detecttargets

function targets=detecttargets(img)

%expects an input image named img 

if ~exist('max_center_dist')
    max_center_dist = 20;
end;
if ~exist('min_size')
    min_size = 5;
end;
if ~exist('max_size')
    max_size = 10000;
end;
if ~exist('min_size_ratio')
    min_size_ratio = 1.0;
end;
if ~exist('max_size_ratio')
    max_size_ratio = 10;
end;


%inverse threshold the input image
%( local adaptive threshold )
imsz= size(img);
im_h=imsz(1);
im_w=imsz(2);
thresh = zeros(im_h,im_w);
blocksize = 100;
for i=1:blocksize:im_w
    for j=1:blocksize:im_h
        i1 = i;
        i2 = min([i1+blocksize im_w]);
        j1 = j;
        j2 = min([j1+blocksize im_h]);
        level = 0.9 * mean(reshape(img(j1:j2, i1:i2), (j2-j1+1)*(i2-i1+1), 1));
        thresh(j1:j2,i1:i2) = img(j1:j2, i1:i2) < max([min([level 100]) 50]);
    end;
end;

%label the contiguous blobs
label = bwlabel(thresh, 8);

%find the regions that are within the right size range
%(discard blobs that are too small or too big)
histogram = hist(label(:),0:max(label(:)));
regions = find( histogram >= min_size & histogram <= max_size ) -1;

regions = regions(find(regions>0));
min_x=repmat(im_w, 1, max(label(:)));
max_x=repmat(0, 1, max(label(:)));
min_y=repmat(im_h, 1, max(label(:)));
max_y=repmat(0, 1, max(label(:)));
for j=1:im_h
    for i=1:im_w
        r=label(j,i);
        if (r>0)
            if (i < min_x(r))
                min_x(r)=i;
            elseif (i > max_x(r))
                max_x(r)=i;     
            end;
            if (j < min_y(r))
                min_y(r)=j;
            elseif (j > max_y(r))
                max_y(r)=j;     
            end;
        end;
    end;
end;
min_x=min_x(regions);
max_x=max_x(regions);
min_y=min_y(regions);
max_y=max_y(regions);
w = max_x - min_x + 1;
h = max_y - min_y + 1;
c_x = min_x + w / 2;
c_y = min_y + h / 2;

temp_x = repmat(c_x, size(c_x,2),1);
temp_y = repmat(c_y, size(c_y,2),1);
dist_x = (temp_x - temp_x');
dist_y = (temp_y - temp_y');
c_dist = sqrt(dist_x.^2 + dist_y.^2);

w_temp = repmat(w, size(w,2),1);
w_ratio = w_temp' ./ w_temp;
h_temp = repmat(h, size(h,2),1);
h_ratio = h_temp' ./ h_temp;

[i j] = find( c_dist < 0.25*w_temp & c_dist < 0.25*h_temp  & (w_ratio > min_size_ratio) & (w_ratio < max_size_ratio) & (h_ratio > min_size_ratio) & (h_ratio < max_size_ratio));
%[i j] = find( (c_dist < w_temp | c_dist < h_temp ) );
icontainsj = (min_x(j) > min_x(i) & max_x(j) < max_x(i) & (min_y(j) > min_y(i)) & max_y(j) < max_y(i));

targets = [];
target_idx = i(icontainsj);
for i=1:length(target_idx)
    targets(i).center_x = c_x(target_idx(i));
    targets(i).center_y = c_y(target_idx(i));
    targets(i).w = w(target_idx(i));
    targets(i).h = h(target_idx(i));
end;

clear temp_x temp_y dist_x dist_y c_dist w_temp w_ratio h_temp h_ratio
clear min_size max_size min_size_ratio max_size_ratio;

%imshow(img); hold on; plot([targets.center_x], [targets.center_y], 'r.');
x=[targets.center_x];
y=[targets.center_y];
K=convhull(x,y);

targets=targets(K(1:end-1));
