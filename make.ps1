Set-PSDebug -Trace 0
$flag=$args[0]

if($flag -eq "--nuget") {
	md dist-nuget\lib\native\ -ea 0
	$source_dir=$pwd
	$version =  python .\useful_files\get_version.py
	
	$target_cnt=@"
<?xml version="1.0" encoding="utf-8"?>
<Project ToolVersion="4.0" xmlns="http://schemas.microsoft.com/developer/msbuild/2003">
  <ItemDefinitionGroup>
    <ClCompile>
      <PreprocessorDefinitions>HAS_LIBSURVIVE;%(PreprocessorDefinitions)</PreprocessorDefinitions>
      <AdditionalIncludeDirectories>`$(MSBuildThisFileDirectory)..\..\lib\native\include;%(AdditionalIncludeDirectories)</AdditionalIncludeDirectories>
    </ClCompile>
  </ItemDefinitionGroup>
  <ItemDefinitionGroup Condition="'`$(Platform)'=='Win32'">
    <Link>
      <AdditionalDependencies>`$(MSBuildThisFileDirectory)..\..\lib\native\lib\win32\libsurvive.dll.a;%(AdditionalDependencies)</AdditionalDependencies>
    </Link>
  </ItemDefinitionGroup>
  <ItemDefinitionGroup Condition="'`$(Platform)'=='x64'">
    <Link>
      <AdditionalDependencies>`$(MSBuildThisFileDirectory)..\..\lib\native\lib\x64\libsurvive.dll.a;%(AdditionalDependencies)</AdditionalDependencies>
    </Link>
  </ItemDefinitionGroup>
  <ItemGroup Condition="'`$(Platform)'=='Win32'">
    <CopyToOutput Include="`$(MSBuildThisFileDirectory)..\..\lib\native\bin\win32\*.dll" />
  </ItemGroup>
  <ItemGroup Condition="'`$(Platform)'=='x64'">
    <CopyToOutput Include="`$(MSBuildThisFileDirectory)..\..\lib\native\bin\x64\*.dll" />
  </ItemGroup>
  <Target Name="OpenBLAS_AfterBuild" AfterTargets="AfterBuild">
    <Copy 
      SkipUnchangedFiles="true" 
      UseHardlinksIfPossible="true"
      SourceFiles="@(CopyToOutput)"  
      DestinationFolder="`$(TargetDir)" />
  </Target>
</Project>
"@
	New-Item "dist-nuget\build\native\libsurvive.targets" -Force
	Set-Content "dist-nuget\build\native\libsurvive.targets" $target_cnt

	$nuspec_cnt=@"
<?xml version="1.0"?>
<package xmlns="http://schemas.microsoft.com/packaging/2011/08/nuspec.xsd">
  <metadata>
	<id>libsurvive.native</id>
	<version>$version</version>
	<title>libsurvive.native</title>
	<owners>jdavidberger</owners>
	<license type="expression">MIT</license>
	<projectUrl>https://github.com/cntools/libsurvive</projectUrl>
	<requireLicenseAcceptance>false</requireLicenseAcceptance>
	<authors>Charles Lohr, Justin Berger</authors>
	<description>Libsurvive is a set of tools and libraries that enable 6 dof tracking on lighthouse and vive based systems that is completely open source and can run on any device. It currently supports both SteamVR 1.0 and SteamVR 2.0 generation of devices and should support any tracked object commercially available.</description>
	<summary>libsurvive native headers and dlls</summary>
	<copyright>Copyright 2020</copyright>
	<tags>native cpp c libsurvive nativepackage</tags>
  </metadata>
  <files>
        <file src="lib\native\bin\x64\**\*.*" target="runtime\win-64\native"     />
		<file src="lib\native\bin\Win32\**\*.*" target="runtime\win-86\native"     />
        <file src="build\**\*.*"   target="build" />
  </files>
</package>
"@
	
	New-Item "dist-nuget\libsurvive.native.spec" -Force
	Set-Content "dist-nuget\libsurvive.native.nuspec" $nuspec_cnt
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
	
	#Compress-Archive -Force -Path .\dist-nuget\* -DestinationPath libsurvive.native.zip
	#mv -Force libsurvive.native.zip libsurvive.native.$version.nupkg
	.\build-win32\nuget.exe pack .\dist-nuget\libsurvive.native.nuspec
} else {
	md build-win -ea 0
	cd build-win
	cmake ..
	cmake --build . --config Release
}