.pushsection command_data, "ax", %progbits
.incbin "data/cyberon_ce_two_stage_model_pack_WithTxtAndMapID.bin"
.popsection

.pushsection license_data, "ax", %progbits
.incbin "data/CybLicense.bin"
.popsection
