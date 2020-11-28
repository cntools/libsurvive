Set-PSDebug -Trace 0
$flag=$args[0]

if($flag -eq "--nuget") {
	md dist-nuget\lib\native\ -ea 0
	$source_dir=$pwd
	$version =  python .\useful_files\get_version.py
	
	md build-win64 -ea 0
	cd build-win64
	cmake "-DCMAKE_INSTALL_PREFIX=$source_dir\dist-nuget\lib\native\" .. -A x64
	cmake --build . --config Release --target install
	
	cd ..
	md build-win32 -ea 0
	cd build-win32
	cmake "-DCMAKE_INSTALL_PREFIX=$source_dir\dist-nuget\lib\native\" .. -A Win32
	cmake --build . --config Release --target install
	cd ..
	
	dotnet build .\bindings\cs\libsurvive.net\libsurvive.net.csproj -c Release
	dotnet msbuild .\bindings\cs\libsurvive.net\libsurvive.net.csproj -t:Pack -p:Configuration=Release -p:Version=$version
} else {
	md build-win -ea 0
	cd build-win
	cmake ..
	cmake --build . --config Release
}