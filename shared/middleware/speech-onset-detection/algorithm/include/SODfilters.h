#ifndef SODFILTERS_H
#define SODFILTERS_H

#define MAX_FIR_ORDER   16

#define D_HALFORDER (8)

extern const int16_t Dc16t[D_HALFORDER / 2];


void firdown2b(int32_t *bc, uint16_t horder, int16_t *x, uint16_t xlen, int16_t *y, int16_t *xmem, int skip, int16_t *ndx);

void firdown2b16(int16_t *bc, uint16_t horder, int16_t *x, uint16_t xlen, int16_t *y, int16_t *xmem, int16_t *ndx);
void firdown2b1632(int16_t *bc, uint16_t horder, int16_t *x, uint16_t xlen, int16_t *y, int16_t *xmem, int16_t *ndx);
void firdown2b16m(int16_t *bc, uint16_t horder, int16_t *x, uint16_t xlen, int16_t *y);
void firdown2b16inline(int16_t *bc, uint16_t horder, int16_t *x, uint16_t xlen, int16_t *y);
void firdown2b16inline_sym(const int16_t* bc, uint16_t horder, int16_t* x, uint16_t xlen, int16_t* y);
void firdown2b16inline_symE(const int16_t* bc, uint16_t horder, int16_t* x, uint16_t xlen, uint32_t *E);

void iir1DCinplace(int16_t *x, uint16_t xlen, int16_t *xmem, int16_t *ymem);
void iir1DCio(int16_t *x, uint16_t xlen, int16_t *y, int16_t *xmem, int16_t *ymem);

uint32_t EnergyTD(int16_t *in, uint16_t len);
uint32_t EnergyTDshift(int16_t *in, uint16_t len, int16_t shift);

void iir1DCinplaceFLOAT(int16_t *x, uint16_t xlen, float *xmem, float *ymem);


#endif
