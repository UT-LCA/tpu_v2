%-------------------------------------------------------------------------
% Matlab script to generate memory contents for loadAddrGen divider_rom
%-------------------------------------------------------------------------
%Parameter
MEMWIDTH_BYTES = 16;
localstride_nbit = log2(MEMWIDTH_BYTES)+1;
maxNElemXfer_nbit = log2(MEMWIDTH_BYTES)+1;

numlanes = input('Please input how many lanes are in the system');

% max_localstride is MEMWIDTH_BYTES-1;
% any greater stride would be "largestride" and use different hardware
max_localstride = MEMWIDTH_BYTES-1;
maxNElemXfer_offsetted_limit = MEMWIDTH_BYTES;

% ROM tables
% Number of elements that can be transferred this cycle
NElemXfer_rom = zeros(1,2^(localstride_nbit+maxNElemXfer_nbit));
% How many elements remain in the current block after the current transfer
NElemRemainder_rom = zeros(1,2^(localstride_nbit+maxNElemXfer_nbit));

%-------------------------------------------------------------------------
% Generate data for ROM table
%-------------------------------------------------------------------------
% iterate through all strides
for stride = 0 : max_localstride
    for maxNElemXfer_offsetted = 0 : maxNElemXfer_offsetted_limit
        % add 1 to index since Matlab arrays start at 1
        ind = maxNElemXfer_offsetted * 2^maxNElemXfer_nbit + stride + 1;
        
		if (ind == 35)
			display('stride');
            stride
            display('maxNElemXfer_offsetted');
            maxNElemXfer_offsetted
		end
		
        % Zero stride case, transfer as quickly as possible
        if (stride == 0)
            NElemXfer_rom(ind) = numlanes;
            NElemRemainder_rom(ind) = 0;
        
        % special case, if number of elements that can be transferred is
        % more than number of lanes * stride, need to limit to numlane
        elseif (maxNElemXfer_offsetted > numlanes*stride)
            NElemXfer_rom(ind) = numlanes;
            % set remainder to be number of elements left to be transferred
            % in the current block
            NElemRemainder_rom(ind) = maxNElemXfer_offsetted - numlanes*stride;

            % display('stride');
            % stride
            % display('maxNElemXfer_offsetted');
            % maxNElemXfer_offsetted
            % display('NElemXfer_rom(ind)');
            % NElemXfer_rom(ind)
            % display('NElemRemainder_rom(ind)');
            % NElemRemainder_rom(ind)
            
        % normal operation
        else
            NElemXfer_rom(ind) = ceil(maxNElemXfer_offsetted / stride);
            NElemRemainder_rom(ind) = mod(maxNElemXfer_offsetted, stride);
        end
    end
end

%-------------------------------------------------------------------------
% Write data to .mif and .dat files
%-------------------------------------------------------------------------
% Write NElemXfer_rom.mif
fid = fopen('NElemXfer_rom.mif','w');
intro_string = '-- Copyright (C) 1991-2007 Altera Corporation,\n -- Your use of Altera Corporations design tools, logic functions \n-- and other software and tools, and its AMPP partner logic \n-- functions, and any output files from any of the foregoing \n-- (including device programming or simulation files), and any \n-- associated documentation or information are expressly subject \n-- to the terms and conditions of the Altera Program License \n-- Subscription Agreement, Altera MegaCore Function License \n-- Agreement, or other applicable license agreement, including, \n-- without limitation, that your use is for the sole purpose of \n-- programming logic devices manufactured by Altera and sold by \n-- Altera or its authorized distributors.  Please refer to the \n-- applicable agreement for further details.';
fprintf(fid,'%s\n',intro_string);
fprintf(fid,'-- Quartus II generated Memory Initialization File (.mif)\n\n');
fprintf(fid,'WIDTH=8;\n');
fprintf(fid,'DEPTH=1024;\n\n');
fprintf(fid,'ADDRESS_RADIX=UNS;\n');
fprintf(fid,'DATA_RADIX=UNS;\n\n');
fprintf(fid,'CONTENT BEGIN\n');

for addr = 0 : 2^(localstride_nbit+maxNElemXfer_nbit)-1
    fprintf(fid,'\t%d\t:\t%d;\n',addr,NElemXfer_rom(addr+1));
end
fprintf(fid,'END;\n');
fclose(fid);

% Write NElemXfer_rom.dat
fid = fopen('NElemXfer_rom.dat','w');
fprintf(fid,'@00000000\n');

for addr = 0 : 2^(localstride_nbit+maxNElemXfer_nbit)-1
    fprintf(fid,'%s\n',dec2hex(NElemXfer_rom(addr+1),2));
end
fclose(fid);

display('NElemXfer_rom.mif written!');
display('NElemXfer_rom.dat written!');

%-------------------------------------------------------
% Write NElemRemainder_rom.mif
%-------------------------------------------------------
fid = fopen('NElemRemainder_rom.mif','w');
fprintf(fid,'%s\n',intro_string);
fprintf(fid,'-- Quartus II generated Memory Initialization File (.mif)\n\n');
fprintf(fid,'WIDTH=8;\n');
fprintf(fid,'DEPTH=1024;\n\n');
fprintf(fid,'ADDRESS_RADIX=UNS;\n');
fprintf(fid,'DATA_RADIX=UNS;\n\n');
fprintf(fid,'CONTENT BEGIN\n');

for addr = 0 : 2^(localstride_nbit+maxNElemXfer_nbit)-1
    fprintf(fid,'\t%d\t:\t%d;\n',addr,NElemRemainder_rom(addr+1));
end
fprintf(fid,'END;\n');
fclose(fid);

% Write NElemRemainder_rom.dat
fid = fopen('NElemRemainder_rom.dat','w');
fprintf(fid,'@00000000\n');

for addr = 0 : 2^(localstride_nbit+maxNElemXfer_nbit)-1
    fprintf(fid,'%s\n',dec2hex(NElemRemainder_rom(addr+1),2));
end
fclose(fid);

display('NElemRemainder_rom.mif written!');
display('NElemRemainder_rom.dat written!');