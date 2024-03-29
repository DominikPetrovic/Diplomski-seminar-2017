/*************************************************************************
Copyright (c) Sergey Bochkanov (ALGLIB project).

>>> SOURCE LICENSE >>>
This  program  is  a  Commercial Edition  of the ALGLIB package  licensed
to Fakultet strojarstva i brodogradnje (Licensee).

Only  Licensee  and  its  Sublicensees  can  use/distribute  it according
to  the  ALGLIB  License  Agreement (see  below) between  Licensor  (Sole
Proprietor Bochkanov Sergey Anatolyevich) and Licensee.

All  developers  working  for  Licensee  should  register  themselves  at
alglib.net. Quick access link for Licensee's account can be found at  the
section 9 of scanned copy of ALGLIB License Agreement or in e-mails  from
ALGLIB Project.

This  source  code  may  contain  modifications  made  by   Licensee   or
Sublicensees  which  fall under the terms of the ALGLIB License Agreement
too.

================ ALGLIB LICENSE AGREEMENT ( APPENDIX A ) ================

DEFINITIONS:
* "ALGLIB" – software delivered by Licensor  to  Licensee  under  present
  Agreement. ALGLIB may include  Binary  Components  (delivered  only  in
  binary form) and Source  Code  Components  (with  optional  precompiled
  binary form).
  ALGLIB  may  include  optional  third-party  components, which may have
  their own licensing terms. Specific list of components  and  additional
  licensing terms (if there are any) is included in the license.txt  file
  in the root of archive containing ALGLIB. If you  decide  not  to  link
  ALGLIB with these optional components, their  licensing  terms  do  not
  apply to you.
* "Application"  -  program  developed  by  Licensee  (either  standalone
  application or software development library) which includes  ALGLIB  as
  one of its parts .
* "Sublicensee"  -  any  party  (including  resellers)   which   receives
  Application from Licensee or another Sublicensee.
* "Application License Agreement"  -  agreement  which   governs   usage/
  redistribution of the Application.
  
LICENSE GRANT:
Subject to the License Restrictions below, Licensor  grants  to  Licensee
the following non-exclusive royalty-free licenses:
A. To modify Source Code Components of ALGLIB and to use modified version
   on the terms of this Agreement.
B. To  develop  Applications  which  use  ALGLIB  and  to distribute such
   Applications in Binary and/or Source Code forms,  with  ALGLIB  either
   statically or dynamically linked. This right is granted provided that:
   * distribution of Source Code forms of Application/ALGLIB is performed
     subject to additional conditions set by clause H (this clause is not
     applied to binary-only distribution)
   * such Applications add significant  primary  functionality  different
     from that of the ALGLIB.
   * such Applications do not expose ALGLIB API (application  programming
     interface) either directly or indirectly
   * Sublicensee  has  no   right   to  use  ALGLIB  except  as  part  of
     the Application
   * any  subsequent  redistribution   respects    conditions    of   the
     present Agreement
   * all developers working for Licensee  should  register  at  company's
     account at www.alglib.net (in order to find login link, see  section
     9 of scanned copy of ALGLIB License Agreement -or find it in e-mails
     from ALGLIB Project).
C. To use Resellers for distribution of the  Application  (in  Binary  or
   Source Code forms), provided that the only activity Reseller  performs
   with Application is redistribution.
   
LICENSE RESTRICTIONS:
D. Licensee/Sublicensee may NOT use, copy or distribute ALGLIB except  as
   provided in this Agreement.
D2. Licensee/Sublicensee may NOT rent or lease ALGLIB to any third party.
E. Licensee/Sublicensee may NOT disassemble, reverse engineer, decompile,
   modify Binary Components of ALGLIB or compiled forms  of  Source  Code
   components.
F. Licensee/Sublicensee  may  NOT  remove  any  copyright notice from the
   Source Code / Binary Components.
G. Licensee/Sublicensee may NOT  disable/remove  code  which  checks  for
   presence of license keys (if such code is included in ALGLIB) from the
   Source Code / Binary Components.
H. Distribution of  Source  Code  forms  of  Application/ALGLIB  must  be
   performed subject to additional conditions:
   * Source Code Components of ALGLIB are distributed only as part of the
     Application. They are not  publicly  distributed.  Sublicensee  must
     explicitly accept Application License Agreement in order  to  access
     ALGLIB source code.
   * Sublicensee has no right to redistribute Application/ALGLIB (in  any
     form, Binary or Source Code), unless Sublicensee is Reseller who  is
     fully compliant with conditions set by clause C.
   * Sublicensee has no right to modify ALGLIB Source  Code,  except  for
     the purpose of fixing bugs
   * Sublicensee has no right to workaround "use ALGLIB only as  part  of
     the Application" limitation by sequentially modifying Application in
     a way which effectively creates new program with different  purpose.
     Application   License  Agreement  may  (a)  explicitly  forbid  such
     modifications, or (b) allow only limited set of "safe" modifications
     (developing plugins, fixing bugs, modifying only specific  parts  of
     the Application).
     
COPYRIGHT:
Title to the ALGLIB and all copies  thereof  remain  with  Licensor.  The
ALGLIB is copyrighted and is protected  by  Russian  copyright  laws  and
international treaty provisions. You will not remove any copyright notice
from the ALGLIB files. You agree to prevent any unauthorized  copying  of
the ALGLIB. Except as expressly provided herein, Licensor does not  grant
any express or implied right to you under Licensor  patents,  copyrights,
trademarks, or trade secret information.
>>> END OF LICENSE >>>
*************************************************************************/
#ifndef _alglibinternal_pkg_h
#define _alglibinternal_pkg_h
#include "ap.h"


/////////////////////////////////////////////////////////////////////////
//
// THIS SECTION CONTAINS COMPUTATIONAL CORE DECLARATIONS (DATATYPES)
//
/////////////////////////////////////////////////////////////////////////
namespace alglib_impl
{
typedef struct
{
    ae_vector ia0;
    ae_vector ia1;
    ae_vector ia2;
    ae_vector ia3;
    ae_vector ra0;
    ae_vector ra1;
    ae_vector ra2;
    ae_vector ra3;
} apbuffers;
typedef struct
{
    ae_bool val;
} sboolean;
typedef struct
{
    ae_vector val;
} sbooleanarray;
typedef struct
{
    ae_int_t val;
} sinteger;
typedef struct
{
    ae_vector val;
} sintegerarray;
typedef struct
{
    double val;
} sreal;
typedef struct
{
    ae_vector val;
} srealarray;
typedef struct
{
    ae_complex val;
} scomplex;
typedef struct
{
    ae_vector val;
} scomplexarray;
typedef struct
{
    ae_int_t chunksize;
    ae_int_t ntotal;
    ae_int_t nin;
    ae_int_t nout;
    ae_int_t wcount;
    ae_vector batch4buf;
    ae_vector hpcbuf;
    ae_matrix xy;
    ae_matrix xy2;
    ae_vector xyrow;
    ae_vector x;
    ae_vector y;
    ae_vector desiredy;
    double e;
    ae_vector g;
    ae_vector tmp0;
} mlpbuffers;
typedef struct
{
    ae_bool brackt;
    ae_bool stage1;
    ae_int_t infoc;
    double dg;
    double dgm;
    double dginit;
    double dgtest;
    double dgx;
    double dgxm;
    double dgy;
    double dgym;
    double finit;
    double ftest1;
    double fm;
    double fx;
    double fxm;
    double fy;
    double fym;
    double stx;
    double sty;
    double stmin;
    double stmax;
    double width;
    double width1;
    double xtrapf;
} linminstate;
typedef struct
{
    ae_bool needf;
    ae_vector x;
    double f;
    ae_int_t n;
    ae_vector xbase;
    ae_vector s;
    double stplen;
    double fcur;
    double stpmax;
    ae_int_t fmax;
    ae_int_t nfev;
    ae_int_t info;
    rcommstate rstate;
} armijostate;
typedef struct
{
    ae_matrix entries;
    ae_vector buffer;
    ae_vector precr;
    ae_vector preci;
    ae_shared_pool bluesteinpool;
} fasttransformplan;

}

/////////////////////////////////////////////////////////////////////////
//
// THIS SECTION CONTAINS C++ INTERFACE
//
/////////////////////////////////////////////////////////////////////////
namespace alglib
{


}

/////////////////////////////////////////////////////////////////////////
//
// THIS SECTION CONTAINS COMPUTATIONAL CORE DECLARATIONS (FUNCTIONS)
//
/////////////////////////////////////////////////////////////////////////
namespace alglib_impl
{
ae_bool seterrorflag(ae_bool* flag, ae_bool cond, ae_state *_state);
ae_bool seterrorflagdiff(ae_bool* flag,
     double val,
     double refval,
     double tol,
     double s,
     ae_state *_state);
void touchint(ae_int_t* a, ae_state *_state);
void touchreal(double* a, ae_state *_state);
double inttoreal(ae_int_t a, ae_state *_state);
double logbase2(double x, ae_state *_state);
ae_bool approxequalrel(double a, double b, double tol, ae_state *_state);
void taskgenint1d(double a,
     double b,
     ae_int_t n,
     /* Real    */ ae_vector* x,
     /* Real    */ ae_vector* y,
     ae_state *_state);
void taskgenint1dequidist(double a,
     double b,
     ae_int_t n,
     /* Real    */ ae_vector* x,
     /* Real    */ ae_vector* y,
     ae_state *_state);
void taskgenint1dcheb1(double a,
     double b,
     ae_int_t n,
     /* Real    */ ae_vector* x,
     /* Real    */ ae_vector* y,
     ae_state *_state);
void taskgenint1dcheb2(double a,
     double b,
     ae_int_t n,
     /* Real    */ ae_vector* x,
     /* Real    */ ae_vector* y,
     ae_state *_state);
ae_bool aredistinct(/* Real    */ ae_vector* x,
     ae_int_t n,
     ae_state *_state);
ae_bool aresameboolean(ae_bool v1, ae_bool v2, ae_state *_state);
void bvectorsetlengthatleast(/* Boolean */ ae_vector* x,
     ae_int_t n,
     ae_state *_state);
void ivectorsetlengthatleast(/* Integer */ ae_vector* x,
     ae_int_t n,
     ae_state *_state);
void rvectorsetlengthatleast(/* Real    */ ae_vector* x,
     ae_int_t n,
     ae_state *_state);
void rmatrixsetlengthatleast(/* Real    */ ae_matrix* x,
     ae_int_t m,
     ae_int_t n,
     ae_state *_state);
void rmatrixresize(/* Real    */ ae_matrix* x,
     ae_int_t m,
     ae_int_t n,
     ae_state *_state);
void imatrixresize(/* Integer */ ae_matrix* x,
     ae_int_t m,
     ae_int_t n,
     ae_state *_state);
ae_bool isfinitevector(/* Real    */ ae_vector* x,
     ae_int_t n,
     ae_state *_state);
ae_bool isfinitecvector(/* Complex */ ae_vector* z,
     ae_int_t n,
     ae_state *_state);
ae_bool apservisfinitematrix(/* Real    */ ae_matrix* x,
     ae_int_t m,
     ae_int_t n,
     ae_state *_state);
ae_bool apservisfinitecmatrix(/* Complex */ ae_matrix* x,
     ae_int_t m,
     ae_int_t n,
     ae_state *_state);
ae_bool isfinitertrmatrix(/* Real    */ ae_matrix* x,
     ae_int_t n,
     ae_bool isupper,
     ae_state *_state);
ae_bool apservisfinitectrmatrix(/* Complex */ ae_matrix* x,
     ae_int_t n,
     ae_bool isupper,
     ae_state *_state);
ae_bool apservisfiniteornanmatrix(/* Real    */ ae_matrix* x,
     ae_int_t m,
     ae_int_t n,
     ae_state *_state);
double safepythag2(double x, double y, ae_state *_state);
double safepythag3(double x, double y, double z, ae_state *_state);
ae_int_t saferdiv(double x, double y, double* r, ae_state *_state);
double safeminposrv(double x, double y, double v, ae_state *_state);
void apperiodicmap(double* x,
     double a,
     double b,
     double* k,
     ae_state *_state);
double randomnormal(ae_state *_state);
void randomunit(ae_int_t n, /* Real    */ ae_vector* x, ae_state *_state);
void inc(ae_int_t* v, ae_state *_state);
void dec(ae_int_t* v, ae_state *_state);
void countdown(ae_int_t* v, ae_state *_state);
double boundval(double x, double b1, double b2, ae_state *_state);
void alloccomplex(ae_serializer* s, ae_complex v, ae_state *_state);
void serializecomplex(ae_serializer* s, ae_complex v, ae_state *_state);
ae_complex unserializecomplex(ae_serializer* s, ae_state *_state);
void allocrealarray(ae_serializer* s,
     /* Real    */ ae_vector* v,
     ae_int_t n,
     ae_state *_state);
void serializerealarray(ae_serializer* s,
     /* Real    */ ae_vector* v,
     ae_int_t n,
     ae_state *_state);
void unserializerealarray(ae_serializer* s,
     /* Real    */ ae_vector* v,
     ae_state *_state);
void allocintegerarray(ae_serializer* s,
     /* Integer */ ae_vector* v,
     ae_int_t n,
     ae_state *_state);
void serializeintegerarray(ae_serializer* s,
     /* Integer */ ae_vector* v,
     ae_int_t n,
     ae_state *_state);
void unserializeintegerarray(ae_serializer* s,
     /* Integer */ ae_vector* v,
     ae_state *_state);
void allocrealmatrix(ae_serializer* s,
     /* Real    */ ae_matrix* v,
     ae_int_t n0,
     ae_int_t n1,
     ae_state *_state);
void serializerealmatrix(ae_serializer* s,
     /* Real    */ ae_matrix* v,
     ae_int_t n0,
     ae_int_t n1,
     ae_state *_state);
void unserializerealmatrix(ae_serializer* s,
     /* Real    */ ae_matrix* v,
     ae_state *_state);
void copyintegerarray(/* Integer */ ae_vector* src,
     /* Integer */ ae_vector* dst,
     ae_state *_state);
void copyrealarray(/* Real    */ ae_vector* src,
     /* Real    */ ae_vector* dst,
     ae_state *_state);
void copyrealmatrix(/* Real    */ ae_matrix* src,
     /* Real    */ ae_matrix* dst,
     ae_state *_state);
ae_int_t recsearch(/* Integer */ ae_vector* a,
     ae_int_t nrec,
     ae_int_t nheader,
     ae_int_t i0,
     ae_int_t i1,
     /* Integer */ ae_vector* b,
     ae_state *_state);
void splitlengtheven(ae_int_t tasksize,
     ae_int_t* task0,
     ae_int_t* task1,
     ae_state *_state);
void splitlength(ae_int_t tasksize,
     ae_int_t chunksize,
     ae_int_t* task0,
     ae_int_t* task1,
     ae_state *_state);
ae_bool _apbuffers_init(void* _p, ae_state *_state, ae_bool make_automatic);
ae_bool _apbuffers_init_copy(void* _dst, void* _src, ae_state *_state, ae_bool make_automatic);
void _apbuffers_clear(void* _p);
void _apbuffers_destroy(void* _p);
ae_bool _sboolean_init(void* _p, ae_state *_state, ae_bool make_automatic);
ae_bool _sboolean_init_copy(void* _dst, void* _src, ae_state *_state, ae_bool make_automatic);
void _sboolean_clear(void* _p);
void _sboolean_destroy(void* _p);
ae_bool _sbooleanarray_init(void* _p, ae_state *_state, ae_bool make_automatic);
ae_bool _sbooleanarray_init_copy(void* _dst, void* _src, ae_state *_state, ae_bool make_automatic);
void _sbooleanarray_clear(void* _p);
void _sbooleanarray_destroy(void* _p);
ae_bool _sinteger_init(void* _p, ae_state *_state, ae_bool make_automatic);
ae_bool _sinteger_init_copy(void* _dst, void* _src, ae_state *_state, ae_bool make_automatic);
void _sinteger_clear(void* _p);
void _sinteger_destroy(void* _p);
ae_bool _sintegerarray_init(void* _p, ae_state *_state, ae_bool make_automatic);
ae_bool _sintegerarray_init_copy(void* _dst, void* _src, ae_state *_state, ae_bool make_automatic);
void _sintegerarray_clear(void* _p);
void _sintegerarray_destroy(void* _p);
ae_bool _sreal_init(void* _p, ae_state *_state, ae_bool make_automatic);
ae_bool _sreal_init_copy(void* _dst, void* _src, ae_state *_state, ae_bool make_automatic);
void _sreal_clear(void* _p);
void _sreal_destroy(void* _p);
ae_bool _srealarray_init(void* _p, ae_state *_state, ae_bool make_automatic);
ae_bool _srealarray_init_copy(void* _dst, void* _src, ae_state *_state, ae_bool make_automatic);
void _srealarray_clear(void* _p);
void _srealarray_destroy(void* _p);
ae_bool _scomplex_init(void* _p, ae_state *_state, ae_bool make_automatic);
ae_bool _scomplex_init_copy(void* _dst, void* _src, ae_state *_state, ae_bool make_automatic);
void _scomplex_clear(void* _p);
void _scomplex_destroy(void* _p);
ae_bool _scomplexarray_init(void* _p, ae_state *_state, ae_bool make_automatic);
ae_bool _scomplexarray_init_copy(void* _dst, void* _src, ae_state *_state, ae_bool make_automatic);
void _scomplexarray_clear(void* _p);
void _scomplexarray_destroy(void* _p);
ae_int_t getrdfserializationcode(ae_state *_state);
ae_int_t getkdtreeserializationcode(ae_state *_state);
ae_int_t getmlpserializationcode(ae_state *_state);
ae_int_t getmlpeserializationcode(ae_state *_state);
ae_int_t getrbfserializationcode(ae_state *_state);
void tagsort(/* Real    */ ae_vector* a,
     ae_int_t n,
     /* Integer */ ae_vector* p1,
     /* Integer */ ae_vector* p2,
     ae_state *_state);
void tagsortbuf(/* Real    */ ae_vector* a,
     ae_int_t n,
     /* Integer */ ae_vector* p1,
     /* Integer */ ae_vector* p2,
     apbuffers* buf,
     ae_state *_state);
void tagsortfasti(/* Real    */ ae_vector* a,
     /* Integer */ ae_vector* b,
     /* Real    */ ae_vector* bufa,
     /* Integer */ ae_vector* bufb,
     ae_int_t n,
     ae_state *_state);
void tagsortfastr(/* Real    */ ae_vector* a,
     /* Real    */ ae_vector* b,
     /* Real    */ ae_vector* bufa,
     /* Real    */ ae_vector* bufb,
     ae_int_t n,
     ae_state *_state);
void tagsortfast(/* Real    */ ae_vector* a,
     /* Real    */ ae_vector* bufa,
     ae_int_t n,
     ae_state *_state);
void tagsortmiddleir(/* Integer */ ae_vector* a,
     /* Real    */ ae_vector* b,
     ae_int_t offset,
     ae_int_t n,
     ae_state *_state);
void tagheappushi(/* Real    */ ae_vector* a,
     /* Integer */ ae_vector* b,
     ae_int_t* n,
     double va,
     ae_int_t vb,
     ae_state *_state);
void tagheapreplacetopi(/* Real    */ ae_vector* a,
     /* Integer */ ae_vector* b,
     ae_int_t n,
     double va,
     ae_int_t vb,
     ae_state *_state);
void tagheappopi(/* Real    */ ae_vector* a,
     /* Integer */ ae_vector* b,
     ae_int_t* n,
     ae_state *_state);
ae_int_t lowerbound(/* Real    */ ae_vector* a,
     ae_int_t n,
     double t,
     ae_state *_state);
ae_int_t upperbound(/* Real    */ ae_vector* a,
     ae_int_t n,
     double t,
     ae_state *_state);
void rankx(/* Real    */ ae_vector* x,
     ae_int_t n,
     ae_bool iscentered,
     apbuffers* buf,
     ae_state *_state);
ae_bool cmatrixrank1f(ae_int_t m,
     ae_int_t n,
     /* Complex */ ae_matrix* a,
     ae_int_t ia,
     ae_int_t ja,
     /* Complex */ ae_vector* u,
     ae_int_t iu,
     /* Complex */ ae_vector* v,
     ae_int_t iv,
     ae_state *_state);
ae_bool rmatrixrank1f(ae_int_t m,
     ae_int_t n,
     /* Real    */ ae_matrix* a,
     ae_int_t ia,
     ae_int_t ja,
     /* Real    */ ae_vector* u,
     ae_int_t iu,
     /* Real    */ ae_vector* v,
     ae_int_t iv,
     ae_state *_state);
ae_bool cmatrixmvf(ae_int_t m,
     ae_int_t n,
     /* Complex */ ae_matrix* a,
     ae_int_t ia,
     ae_int_t ja,
     ae_int_t opa,
     /* Complex */ ae_vector* x,
     ae_int_t ix,
     /* Complex */ ae_vector* y,
     ae_int_t iy,
     ae_state *_state);
ae_bool rmatrixmvf(ae_int_t m,
     ae_int_t n,
     /* Real    */ ae_matrix* a,
     ae_int_t ia,
     ae_int_t ja,
     ae_int_t opa,
     /* Real    */ ae_vector* x,
     ae_int_t ix,
     /* Real    */ ae_vector* y,
     ae_int_t iy,
     ae_state *_state);
ae_bool cmatrixrighttrsmf(ae_int_t m,
     ae_int_t n,
     /* Complex */ ae_matrix* a,
     ae_int_t i1,
     ae_int_t j1,
     ae_bool isupper,
     ae_bool isunit,
     ae_int_t optype,
     /* Complex */ ae_matrix* x,
     ae_int_t i2,
     ae_int_t j2,
     ae_state *_state);
ae_bool cmatrixlefttrsmf(ae_int_t m,
     ae_int_t n,
     /* Complex */ ae_matrix* a,
     ae_int_t i1,
     ae_int_t j1,
     ae_bool isupper,
     ae_bool isunit,
     ae_int_t optype,
     /* Complex */ ae_matrix* x,
     ae_int_t i2,
     ae_int_t j2,
     ae_state *_state);
ae_bool rmatrixrighttrsmf(ae_int_t m,
     ae_int_t n,
     /* Real    */ ae_matrix* a,
     ae_int_t i1,
     ae_int_t j1,
     ae_bool isupper,
     ae_bool isunit,
     ae_int_t optype,
     /* Real    */ ae_matrix* x,
     ae_int_t i2,
     ae_int_t j2,
     ae_state *_state);
ae_bool rmatrixlefttrsmf(ae_int_t m,
     ae_int_t n,
     /* Real    */ ae_matrix* a,
     ae_int_t i1,
     ae_int_t j1,
     ae_bool isupper,
     ae_bool isunit,
     ae_int_t optype,
     /* Real    */ ae_matrix* x,
     ae_int_t i2,
     ae_int_t j2,
     ae_state *_state);
ae_bool cmatrixsyrkf(ae_int_t n,
     ae_int_t k,
     double alpha,
     /* Complex */ ae_matrix* a,
     ae_int_t ia,
     ae_int_t ja,
     ae_int_t optypea,
     double beta,
     /* Complex */ ae_matrix* c,
     ae_int_t ic,
     ae_int_t jc,
     ae_bool isupper,
     ae_state *_state);
ae_bool rmatrixsyrkf(ae_int_t n,
     ae_int_t k,
     double alpha,
     /* Real    */ ae_matrix* a,
     ae_int_t ia,
     ae_int_t ja,
     ae_int_t optypea,
     double beta,
     /* Real    */ ae_matrix* c,
     ae_int_t ic,
     ae_int_t jc,
     ae_bool isupper,
     ae_state *_state);
ae_bool rmatrixgemmf(ae_int_t m,
     ae_int_t n,
     ae_int_t k,
     double alpha,
     /* Real    */ ae_matrix* a,
     ae_int_t ia,
     ae_int_t ja,
     ae_int_t optypea,
     /* Real    */ ae_matrix* b,
     ae_int_t ib,
     ae_int_t jb,
     ae_int_t optypeb,
     double beta,
     /* Real    */ ae_matrix* c,
     ae_int_t ic,
     ae_int_t jc,
     ae_state *_state);
ae_bool cmatrixgemmf(ae_int_t m,
     ae_int_t n,
     ae_int_t k,
     ae_complex alpha,
     /* Complex */ ae_matrix* a,
     ae_int_t ia,
     ae_int_t ja,
     ae_int_t optypea,
     /* Complex */ ae_matrix* b,
     ae_int_t ib,
     ae_int_t jb,
     ae_int_t optypeb,
     ae_complex beta,
     /* Complex */ ae_matrix* c,
     ae_int_t ic,
     ae_int_t jc,
     ae_state *_state);
void cmatrixgemmk(ae_int_t m,
     ae_int_t n,
     ae_int_t k,
     ae_complex alpha,
     /* Complex */ ae_matrix* a,
     ae_int_t ia,
     ae_int_t ja,
     ae_int_t optypea,
     /* Complex */ ae_matrix* b,
     ae_int_t ib,
     ae_int_t jb,
     ae_int_t optypeb,
     ae_complex beta,
     /* Complex */ ae_matrix* c,
     ae_int_t ic,
     ae_int_t jc,
     ae_state *_state);
void rmatrixgemmk(ae_int_t m,
     ae_int_t n,
     ae_int_t k,
     double alpha,
     /* Real    */ ae_matrix* a,
     ae_int_t ia,
     ae_int_t ja,
     ae_int_t optypea,
     /* Real    */ ae_matrix* b,
     ae_int_t ib,
     ae_int_t jb,
     ae_int_t optypeb,
     double beta,
     /* Real    */ ae_matrix* c,
     ae_int_t ic,
     ae_int_t jc,
     ae_state *_state);
void rmatrixgemmk44v00(ae_int_t m,
     ae_int_t n,
     ae_int_t k,
     double alpha,
     /* Real    */ ae_matrix* a,
     ae_int_t ia,
     ae_int_t ja,
     /* Real    */ ae_matrix* b,
     ae_int_t ib,
     ae_int_t jb,
     double beta,
     /* Real    */ ae_matrix* c,
     ae_int_t ic,
     ae_int_t jc,
     ae_state *_state);
void rmatrixgemmk44v01(ae_int_t m,
     ae_int_t n,
     ae_int_t k,
     double alpha,
     /* Real    */ ae_matrix* a,
     ae_int_t ia,
     ae_int_t ja,
     /* Real    */ ae_matrix* b,
     ae_int_t ib,
     ae_int_t jb,
     double beta,
     /* Real    */ ae_matrix* c,
     ae_int_t ic,
     ae_int_t jc,
     ae_state *_state);
void rmatrixgemmk44v10(ae_int_t m,
     ae_int_t n,
     ae_int_t k,
     double alpha,
     /* Real    */ ae_matrix* a,
     ae_int_t ia,
     ae_int_t ja,
     /* Real    */ ae_matrix* b,
     ae_int_t ib,
     ae_int_t jb,
     double beta,
     /* Real    */ ae_matrix* c,
     ae_int_t ic,
     ae_int_t jc,
     ae_state *_state);
void rmatrixgemmk44v11(ae_int_t m,
     ae_int_t n,
     ae_int_t k,
     double alpha,
     /* Real    */ ae_matrix* a,
     ae_int_t ia,
     ae_int_t ja,
     /* Real    */ ae_matrix* b,
     ae_int_t ib,
     ae_int_t jb,
     double beta,
     /* Real    */ ae_matrix* c,
     ae_int_t ic,
     ae_int_t jc,
     ae_state *_state);
ae_bool rmatrixsyrkmkl(ae_int_t n,
     ae_int_t k,
     double alpha,
     /* Real    */ ae_matrix* a,
     ae_int_t ia,
     ae_int_t ja,
     ae_int_t optypea,
     double beta,
     /* Real    */ ae_matrix* c,
     ae_int_t ic,
     ae_int_t jc,
     ae_bool isupper,
     ae_state *_state);
ae_bool rmatrixgemmmkl(ae_int_t m,
     ae_int_t n,
     ae_int_t k,
     double alpha,
     /* Real    */ ae_matrix* a,
     ae_int_t ia,
     ae_int_t ja,
     ae_int_t optypea,
     /* Real    */ ae_matrix* b,
     ae_int_t ib,
     ae_int_t jb,
     ae_int_t optypeb,
     double beta,
     /* Real    */ ae_matrix* c,
     ae_int_t ic,
     ae_int_t jc,
     ae_state *_state);
double vectornorm2(/* Real    */ ae_vector* x,
     ae_int_t i1,
     ae_int_t i2,
     ae_state *_state);
ae_int_t vectoridxabsmax(/* Real    */ ae_vector* x,
     ae_int_t i1,
     ae_int_t i2,
     ae_state *_state);
ae_int_t columnidxabsmax(/* Real    */ ae_matrix* x,
     ae_int_t i1,
     ae_int_t i2,
     ae_int_t j,
     ae_state *_state);
ae_int_t rowidxabsmax(/* Real    */ ae_matrix* x,
     ae_int_t j1,
     ae_int_t j2,
     ae_int_t i,
     ae_state *_state);
double upperhessenberg1norm(/* Real    */ ae_matrix* a,
     ae_int_t i1,
     ae_int_t i2,
     ae_int_t j1,
     ae_int_t j2,
     /* Real    */ ae_vector* work,
     ae_state *_state);
void copymatrix(/* Real    */ ae_matrix* a,
     ae_int_t is1,
     ae_int_t is2,
     ae_int_t js1,
     ae_int_t js2,
     /* Real    */ ae_matrix* b,
     ae_int_t id1,
     ae_int_t id2,
     ae_int_t jd1,
     ae_int_t jd2,
     ae_state *_state);
void inplacetranspose(/* Real    */ ae_matrix* a,
     ae_int_t i1,
     ae_int_t i2,
     ae_int_t j1,
     ae_int_t j2,
     /* Real    */ ae_vector* work,
     ae_state *_state);
void copyandtranspose(/* Real    */ ae_matrix* a,
     ae_int_t is1,
     ae_int_t is2,
     ae_int_t js1,
     ae_int_t js2,
     /* Real    */ ae_matrix* b,
     ae_int_t id1,
     ae_int_t id2,
     ae_int_t jd1,
     ae_int_t jd2,
     ae_state *_state);
void matrixvectormultiply(/* Real    */ ae_matrix* a,
     ae_int_t i1,
     ae_int_t i2,
     ae_int_t j1,
     ae_int_t j2,
     ae_bool trans,
     /* Real    */ ae_vector* x,
     ae_int_t ix1,
     ae_int_t ix2,
     double alpha,
     /* Real    */ ae_vector* y,
     ae_int_t iy1,
     ae_int_t iy2,
     double beta,
     ae_state *_state);
double pythag2(double x, double y, ae_state *_state);
void matrixmatrixmultiply(/* Real    */ ae_matrix* a,
     ae_int_t ai1,
     ae_int_t ai2,
     ae_int_t aj1,
     ae_int_t aj2,
     ae_bool transa,
     /* Real    */ ae_matrix* b,
     ae_int_t bi1,
     ae_int_t bi2,
     ae_int_t bj1,
     ae_int_t bj2,
     ae_bool transb,
     double alpha,
     /* Real    */ ae_matrix* c,
     ae_int_t ci1,
     ae_int_t ci2,
     ae_int_t cj1,
     ae_int_t cj2,
     double beta,
     /* Real    */ ae_vector* work,
     ae_state *_state);
void hermitianmatrixvectormultiply(/* Complex */ ae_matrix* a,
     ae_bool isupper,
     ae_int_t i1,
     ae_int_t i2,
     /* Complex */ ae_vector* x,
     ae_complex alpha,
     /* Complex */ ae_vector* y,
     ae_state *_state);
void hermitianrank2update(/* Complex */ ae_matrix* a,
     ae_bool isupper,
     ae_int_t i1,
     ae_int_t i2,
     /* Complex */ ae_vector* x,
     /* Complex */ ae_vector* y,
     /* Complex */ ae_vector* t,
     ae_complex alpha,
     ae_state *_state);
void generatereflection(/* Real    */ ae_vector* x,
     ae_int_t n,
     double* tau,
     ae_state *_state);
void applyreflectionfromtheleft(/* Real    */ ae_matrix* c,
     double tau,
     /* Real    */ ae_vector* v,
     ae_int_t m1,
     ae_int_t m2,
     ae_int_t n1,
     ae_int_t n2,
     /* Real    */ ae_vector* work,
     ae_state *_state);
void applyreflectionfromtheright(/* Real    */ ae_matrix* c,
     double tau,
     /* Real    */ ae_vector* v,
     ae_int_t m1,
     ae_int_t m2,
     ae_int_t n1,
     ae_int_t n2,
     /* Real    */ ae_vector* work,
     ae_state *_state);
void complexgeneratereflection(/* Complex */ ae_vector* x,
     ae_int_t n,
     ae_complex* tau,
     ae_state *_state);
void complexapplyreflectionfromtheleft(/* Complex */ ae_matrix* c,
     ae_complex tau,
     /* Complex */ ae_vector* v,
     ae_int_t m1,
     ae_int_t m2,
     ae_int_t n1,
     ae_int_t n2,
     /* Complex */ ae_vector* work,
     ae_state *_state);
void complexapplyreflectionfromtheright(/* Complex */ ae_matrix* c,
     ae_complex tau,
     /* Complex */ ae_vector* v,
     ae_int_t m1,
     ae_int_t m2,
     ae_int_t n1,
     ae_int_t n2,
     /* Complex */ ae_vector* work,
     ae_state *_state);
void symmetricmatrixvectormultiply(/* Real    */ ae_matrix* a,
     ae_bool isupper,
     ae_int_t i1,
     ae_int_t i2,
     /* Real    */ ae_vector* x,
     double alpha,
     /* Real    */ ae_vector* y,
     ae_state *_state);
void symmetricrank2update(/* Real    */ ae_matrix* a,
     ae_bool isupper,
     ae_int_t i1,
     ae_int_t i2,
     /* Real    */ ae_vector* x,
     /* Real    */ ae_vector* y,
     /* Real    */ ae_vector* t,
     double alpha,
     ae_state *_state);
void applyrotationsfromtheleft(ae_bool isforward,
     ae_int_t m1,
     ae_int_t m2,
     ae_int_t n1,
     ae_int_t n2,
     /* Real    */ ae_vector* c,
     /* Real    */ ae_vector* s,
     /* Real    */ ae_matrix* a,
     /* Real    */ ae_vector* work,
     ae_state *_state);
void applyrotationsfromtheright(ae_bool isforward,
     ae_int_t m1,
     ae_int_t m2,
     ae_int_t n1,
     ae_int_t n2,
     /* Real    */ ae_vector* c,
     /* Real    */ ae_vector* s,
     /* Real    */ ae_matrix* a,
     /* Real    */ ae_vector* work,
     ae_state *_state);
void generaterotation(double f,
     double g,
     double* cs,
     double* sn,
     double* r,
     ae_state *_state);
ae_bool upperhessenbergschurdecomposition(/* Real    */ ae_matrix* h,
     ae_int_t n,
     /* Real    */ ae_matrix* s,
     ae_state *_state);
void internalschurdecomposition(/* Real    */ ae_matrix* h,
     ae_int_t n,
     ae_int_t tneeded,
     ae_int_t zneeded,
     /* Real    */ ae_vector* wr,
     /* Real    */ ae_vector* wi,
     /* Real    */ ae_matrix* z,
     ae_int_t* info,
     ae_state *_state);
void rmatrixtrsafesolve(/* Real    */ ae_matrix* a,
     ae_int_t n,
     /* Real    */ ae_vector* x,
     double* s,
     ae_bool isupper,
     ae_bool istrans,
     ae_bool isunit,
     ae_state *_state);
void safesolvetriangular(/* Real    */ ae_matrix* a,
     ae_int_t n,
     /* Real    */ ae_vector* x,
     double* s,
     ae_bool isupper,
     ae_bool istrans,
     ae_bool isunit,
     ae_bool normin,
     /* Real    */ ae_vector* cnorm,
     ae_state *_state);
ae_bool rmatrixscaledtrsafesolve(/* Real    */ ae_matrix* a,
     double sa,
     ae_int_t n,
     /* Real    */ ae_vector* x,
     ae_bool isupper,
     ae_int_t trans,
     ae_bool isunit,
     double maxgrowth,
     ae_state *_state);
ae_bool cmatrixscaledtrsafesolve(/* Complex */ ae_matrix* a,
     double sa,
     ae_int_t n,
     /* Complex */ ae_vector* x,
     ae_bool isupper,
     ae_int_t trans,
     ae_bool isunit,
     double maxgrowth,
     ae_state *_state);
void hpcpreparechunkedgradient(/* Real    */ ae_vector* weights,
     ae_int_t wcount,
     ae_int_t ntotal,
     ae_int_t nin,
     ae_int_t nout,
     mlpbuffers* buf,
     ae_state *_state);
void hpcfinalizechunkedgradient(mlpbuffers* buf,
     /* Real    */ ae_vector* grad,
     ae_state *_state);
ae_bool hpcchunkedgradient(/* Real    */ ae_vector* weights,
     /* Integer */ ae_vector* structinfo,
     /* Real    */ ae_vector* columnmeans,
     /* Real    */ ae_vector* columnsigmas,
     /* Real    */ ae_matrix* xy,
     ae_int_t cstart,
     ae_int_t csize,
     /* Real    */ ae_vector* batch4buf,
     /* Real    */ ae_vector* hpcbuf,
     double* e,
     ae_bool naturalerrorfunc,
     ae_state *_state);
ae_bool hpcchunkedprocess(/* Real    */ ae_vector* weights,
     /* Integer */ ae_vector* structinfo,
     /* Real    */ ae_vector* columnmeans,
     /* Real    */ ae_vector* columnsigmas,
     /* Real    */ ae_matrix* xy,
     ae_int_t cstart,
     ae_int_t csize,
     /* Real    */ ae_vector* batch4buf,
     /* Real    */ ae_vector* hpcbuf,
     ae_state *_state);
ae_bool _mlpbuffers_init(void* _p, ae_state *_state, ae_bool make_automatic);
ae_bool _mlpbuffers_init_copy(void* _dst, void* _src, ae_state *_state, ae_bool make_automatic);
void _mlpbuffers_clear(void* _p);
void _mlpbuffers_destroy(void* _p);
void xdot(/* Real    */ ae_vector* a,
     /* Real    */ ae_vector* b,
     ae_int_t n,
     /* Real    */ ae_vector* temp,
     double* r,
     double* rerr,
     ae_state *_state);
void xcdot(/* Complex */ ae_vector* a,
     /* Complex */ ae_vector* b,
     ae_int_t n,
     /* Real    */ ae_vector* temp,
     ae_complex* r,
     double* rerr,
     ae_state *_state);
void linminnormalized(/* Real    */ ae_vector* d,
     double* stp,
     ae_int_t n,
     ae_state *_state);
void mcsrch(ae_int_t n,
     /* Real    */ ae_vector* x,
     double* f,
     /* Real    */ ae_vector* g,
     /* Real    */ ae_vector* s,
     double* stp,
     double stpmax,
     double gtol,
     ae_int_t* info,
     ae_int_t* nfev,
     /* Real    */ ae_vector* wa,
     linminstate* state,
     ae_int_t* stage,
     ae_state *_state);
void armijocreate(ae_int_t n,
     /* Real    */ ae_vector* x,
     double f,
     /* Real    */ ae_vector* s,
     double stp,
     double stpmax,
     ae_int_t fmax,
     armijostate* state,
     ae_state *_state);
ae_bool armijoiteration(armijostate* state, ae_state *_state);
void armijoresults(armijostate* state,
     ae_int_t* info,
     double* stp,
     double* f,
     ae_state *_state);
ae_bool _linminstate_init(void* _p, ae_state *_state, ae_bool make_automatic);
ae_bool _linminstate_init_copy(void* _dst, void* _src, ae_state *_state, ae_bool make_automatic);
void _linminstate_clear(void* _p);
void _linminstate_destroy(void* _p);
ae_bool _armijostate_init(void* _p, ae_state *_state, ae_bool make_automatic);
ae_bool _armijostate_init_copy(void* _dst, void* _src, ae_state *_state, ae_bool make_automatic);
void _armijostate_clear(void* _p);
void _armijostate_destroy(void* _p);
void findprimitiverootandinverse(ae_int_t n,
     ae_int_t* proot,
     ae_int_t* invproot,
     ae_state *_state);
void ftcomplexfftplan(ae_int_t n,
     ae_int_t k,
     fasttransformplan* plan,
     ae_state *_state);
void ftapplyplan(fasttransformplan* plan,
     /* Real    */ ae_vector* a,
     ae_int_t offsa,
     ae_int_t repcnt,
     ae_state *_state);
void ftbasefactorize(ae_int_t n,
     ae_int_t tasktype,
     ae_int_t* n1,
     ae_int_t* n2,
     ae_state *_state);
ae_bool ftbaseissmooth(ae_int_t n, ae_state *_state);
ae_int_t ftbasefindsmooth(ae_int_t n, ae_state *_state);
ae_int_t ftbasefindsmootheven(ae_int_t n, ae_state *_state);
double ftbasegetflopestimate(ae_int_t n, ae_state *_state);
ae_bool _fasttransformplan_init(void* _p, ae_state *_state, ae_bool make_automatic);
ae_bool _fasttransformplan_init_copy(void* _dst, void* _src, ae_state *_state, ae_bool make_automatic);
void _fasttransformplan_clear(void* _p);
void _fasttransformplan_destroy(void* _p);
double nulog1p(double x, ae_state *_state);
double nuexpm1(double x, ae_state *_state);
double nucosm1(double x, ae_state *_state);

}
#endif

