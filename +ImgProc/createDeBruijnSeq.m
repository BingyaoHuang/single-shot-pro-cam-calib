function [horiList, vertList, horiPos, vertPos] = createDeBruijnSeq(prjW, prjH)
%% Create the De Bruijn sequence we used in the paper.
% 1 = Red
% 2 = Yellow
% 3 = Lime
% 4 = Green
% 5 = Cyan
% 6 = Blue
% 7 = Purple
% 8 = Magenta
% See also: ImgProc.genStructuredLight

%% License
% ACADEMIC OR NON-PROFIT ORGANIZATION NONCOMMERCIAL RESEARCH USE ONLY
% Copyright (c) 2018 Bingyao Huang
% All rights reserved.

% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met:

% The above copyright notice and this permission notice shall be included in all
% copies or substantial portions of the Software.

% If you publish results obtained using this software, please cite our paper.

% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.

%%

HStripeT = 2;
VStripeT = 2;
TopBotbuffer = 10;
LeftRightbuffer = 10;
Debrujin_k = 4; % of color
Debrujin_n = 3; %size of window (seq length)
Debrujin_size = power(Debrujin_k, Debrujin_n)+(Debrujin_n-1); %length of Debruijn sequence, +n-1 for wrap

Hsr = TopBotbuffer; % start position of Horizontal strips
Vsr = LeftRightbuffer; % start position of Vertical strips
Hspace = floor((prjH - TopBotbuffer * 2 - HStripeT) / (Debrujin_size-1));  % has to match CreateDeBrujin
Vspace = floor((prjW - LeftRightbuffer * 2 - VStripeT) / (Debrujin_size-1));

% Create Debruijn sequence
sequence = zeros(Debrujin_size,1);

HL = 4;

digOne = HL - 1;
digTwo = HL;
i = 0;
for curHL = 1 : HL
    for curdigOne = 1:digOne
        for curdigTwo = 1:digTwo
            i = i+1;
            sequence(i) = (HL - (curdigOne - 1));
            i = i+1;
            sequence(i) = (HL - (curdigTwo - 1));
            i = i+1;
            sequence(i) = (curHL);
        end
    end
    
    if (curHL < HL)
        i = i+1;
        sequence(i) = (curHL);
    end
    
    digOne = digOne - 1;
    digTwo = digTwo - 1;
    
end

% Three additions of HL to the end
i = i+1;
sequence(i) = (HL);
i = i+1;
sequence(i) = (HL);
i = i+1;
sequence(i) = (HL);

horiList = zeros(Debrujin_size,1);
vertList = zeros(Debrujin_size,1);
horiPos = zeros(Debrujin_size,1);
vertPos = zeros(Debrujin_size,1);

% Create De Bruijn look up table(position and list)
for i = 1:Debrujin_size
    horiList(i) = 2 * sequence(i) - 1;
    horiPos(i)  = Hsr+2; % for 1280 and topbottom buffer = 20, space is 19.375 without HstripT
    Hsr = Hsr + Hspace;
    
    %Vertical
    vertList(i)  = 2 * sequence(i);
    vertPos(i)  = Vsr+2; % for 1280 and topbottom buffer = 20, space is 19.375 without HstripT
    Vsr = Vsr + Vspace;
end

horiList = horiList';
vertList = vertList';
horiPos = horiPos';
vertPos = vertPos';

end


%     horiList = [7,7,1,7,5,1,7,3,1,7,1,1,5,7,1,5,5,1,5,3,1,5,1,1,3,7,1,3,5,1,...
%         3,3,1,3,1,1,1,7,7,3,7,5,3,7,3,3,5,7,3,5,5,3,5,3,3,3,7,7,5,7,5,5,5,7];
%
%     vertList = [8,8,2,8,6,2,8,4,2,8,2,2,6,8,2,6,6,2,6,4,2,6,2,2,4,8,2,4,6,2,...
%         4,4,2,4,2,2,2,8,8,4,8,6,4,8,4,4,6,8,4,6,6,4,6,4,4,4,8,8,6,8,6,6,6,8];
%
%     horiPos = [10,21,32,43,54,65,76,87,98,109,120,131,142,153,164,175,186,197,...
%         208,219,230,241,252,263,274,285,296,307,318,329,340,351,362,373,384,395,...
%         406,417,428,439,450,461,472,483,494,505,516,527,538,549,560,571,582,593,604,...
%         615,626,637,648,659,670,681,692,703];
%
%     vertPos = [10,29,48,67,86,105,124,143,162,181,200,219,238,257,276,295,314,333,...
%         352,371,390,409,428,447,466,485,504,523,542,561,580,599,618,637,656,675,...
%         694,713,732,751,770,789,808,827,846,865,884,903,922,941,960,979,998,1017,...
%         1036,1055,1074,1093,1112,1131,1150,1169,1188,1207];

