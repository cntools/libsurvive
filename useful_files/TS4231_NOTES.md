Based on u_cap's initial findings...

Here's what I see
 * The Github default of 0x392B lets me see reasonable (if noisy) E and D signals with an HTC base in default settings
 * The 0x3FFF that I thought I had read back from a power-cycled TS4231 shows the same when set explicitly: E and D present. I probably did not set the bus up properly for reading the config(edited)
 * Using 0x2BA8 I get D==0 and E==1 , no changes
 * configDevice has an initial section that tries to get in S3_STATE before setting a config
 * if I take that out in its entirety, we are in S0_STATE, and reading back the config will get 0x3FFF - I suspect that is a bogus result(edited)
 * if Ileave the bus setup section in and read (but not write) the config after S3_STATE has been reached, I get a default of 0x0 read(edited)
 * this default state will also show me E==1 and D==0, no changes(edited)
 * same result if I set 0 explicitly - apparently a valid config, but not working with HTC bases(edited)
 * summary: If I set the GitHub default config, I get E+D and everything works fine. If I set 0x3FFF, same result
 * If I set 0x2BA8, I D and E show no changes ever with HTC base. same if I set 0, or if I just power cycle and do not set a config, in which case the config reads as 0 as long as I read in S3_STATE
 * I'll try different frequencies w/ 0x2BA8  and 0 next. the behavior of the default/zero config makes no sense as a legacy Gen1 support, so maybe its a Gen2 default?
 * I never see E-only
 * on a related note, I need to find a better way to power-cycle the TS4231 from a Teensy
 * right now I power-cycle both.

Later on...

 * well, I can get the thing to work - pulses on E and D - in a number of ways, but now that I tried your "do nothing, just read", the question becomes what of all that machinery exists really for the non-default modes
 * goToWatch? nah, not needed, but JSeibel (the Triad guy who wrote this) does it all the time
 * write_config? not needed, but the Triad code does it anyway
 * the bus setup? only needed if you want to read back the config word, apparently
 
