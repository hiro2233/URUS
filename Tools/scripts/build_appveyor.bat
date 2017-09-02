goto %TOOLSET%

:cygwin

move C:\cygwin C:\cygwin_bk
move C:\cygwin64 C:\cygwin
C:\cygwin\setup-x86_64.exe -qnNdO -R C:\cygwin -s http://cygwin.mirror.constant.com -l C:\cygwin\var\cache\setup -P gcc-g++,gcc-core,binutils,libxml2,libxml2-devel,libxslt-devel,python2,python2-devel,python2-pip,git,ccache
path C:\cygwin\bin;%path%
set CHERE_INVOKING=yes
bash -lc "pip2 install empy pyserial future lxml"
bash -lc "pip2 install mavproxy"
bash -lc "git submodule sync --recursive"
bash -lc "git submodule foreach --recursive 'git fetch --tags'"
bash -lc "git submodule update --init --recursive"
bash -lc "./waf configure --board=sitl && ./waf clean && ./waf bin"

goto :eof
