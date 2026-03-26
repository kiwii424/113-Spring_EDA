
/* =========FOR INTERNAL USE ONLY. NO DISTRIBUTION PLEASE ========== */

/*********************************************************************
 Copyright 2002-2006, University of California at Santa Barbara (UCSB).
 All rights reserved.

 --- This software and any associated documentation is provided "as is"

 UCSB and the authors MAKES NO REPRESENTATIONS OR WARRANTIES, EXPRESS
 OR IMPLIED, INCLUDING THOSE OF MERCHANTABILITY OR FITNESS FOR A
 PARTICULAR PURPOSE, OR THAT  USE OF THE SOFTWARE, MODIFICATIONS, OR
 ASSOCIATED DOCUMENTATION WILL NOT INFRINGE ANY PATENTS, COPYRIGHTS,
 TRADEMARKS OR OTHER INTELLECTUAL PROPERTY RIGHTS OF A THIRD PARTY.

 UCSB and the authors shall not be liable under any circumstances for
 any direct, indirect, special, incidental, or consequential damages
 with respect to any claim by USER or any third party on account of
 or arising from the use, or inability to use, this software or its
 associated documentation, even if UCSB and/or the authors has been advised
 of the possibility of those damages.
*********************************************************************/
// Author: Kai Yang (kyang@ece.ucsb.edu)

using namespace std;

#include <string.h>
#include <cstdlib>
#include <assert.h>
#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include "GetLongOpt.h"
#include "kai_gate.h"
#include "kai_path.h"
#include "kai_objective.h"
#include "kai_typeemu.h"

extern GetLongOpt option;
extern map<string, KaiGATE *> CUTNameTable;

string ToCUTName(KaiGATE *gptr, int TF)
{
	char tmp[100];
	sprintf(tmp, "%d", TF);
	string dupname;
	if (gptr->GetFunction() != G_DFF)
		dupname = gptr->GetName() + "_dup_t" + tmp;
	else
		dupname = gptr->GetName() + "_ppi_t" + tmp;
	KaiGATE *tg = CUTNameTable[dupname];
	if (!tg)
	{
		cerr << "Miss ToCUTName()" << endl;
		exit(-1);
	}
	return dupname;
}

void AtpgObj::BuildFromPath_NR(PATH *pptr)
{
	cleanup();
	KaiGATE *CurG, *PreG;
	TRANSITION CurT, PreT;
	assert(pptr->NoGate() == pptr->NoTrans());

	PreG = pptr->GetGate(0);  /* input gate on sensitive path*/
	PreT = pptr->GetTrans(0); /* input transition on sensitive path*/

	if (PreT == R || PreT == F)
	{
		AddObj(ToCUTName(PreG, 0), PreT == R ? 0 : 1);
		AddObj(ToCUTName(PreG, 1), PreT == R ? 1 : 0);
	}
	else
	{
		cerr << "R/F Error !" << endl;
		exit(-1);
	}

	/*Fault Propagation = off-input setting on sensitive path */
	for (int i = 0; i < pptr->NoGate(); ++i)
	{
		CurG = pptr->GetGate(i);
		CurT = pptr->GetTrans(i);

		const bool isAndFamily = (CurG->GetFunction() == G_AND ||
								  CurG->GetFunction() == G_NAND);
		const int NCV = isAndFamily ? 1 : 0; // AND/NAND→1, OR/NOR→0

		if (CurG->GetFunction() == G_PI)
		{
			if (CurT == R || CurT == F)
			{
				AddObj(ToCUTName(CurG, 0), CurT == R ? 0 : 1);
				AddObj(ToCUTName(CurG, 1), CurT == R ? 1 : 0);
			}
			else
			{
				cerr << "R/F Error !" << endl;
				exit(-1);
			}
		}
		else if (PreT == R || PreT == F)
		{
			for (int j = 0; j < CurG->NoFanin(); ++j)
			{
				if (CurG->Fanin(j) != PreG)
				{
					AddObj(ToCUTName(CurG->Fanin(j), 1), NCV);
				}
			}
		}
		else
		{
			cerr << "R/F Error !" << endl;
			exit(-1);
		}

		PreG = CurG;
		PreT = CurT;
	}
}


void AtpgObj::BuildFromPath_R(PATH *pptr) {
	cleanup();
	KaiGATE *CurG, *PreG;
	TRANSITION CurT, PreT;
	assert(pptr->NoGate() == pptr->NoTrans());

	for (int i = 0; i < pptr->NoGate(); ++i) {
		CurG = pptr->GetGate(i);
		CurT = pptr->GetTrans(i);

		const bool isAndFamily = (CurG->GetFunction() == G_AND ||
								  CurG->GetFunction() == G_NAND);
		const int NCV = isAndFamily ? 1 : 0; // AND/NAND →1, OR/NOR →0

		if (CurG->GetFunction() == G_PI) {
			/* Path node is a primary input */
			if (CurT == R || CurT == F) {
				AddObj(ToCUTName(CurG, 0), CurT == R ? 0 : 1);
				AddObj(ToCUTName(CurG, 1), CurT == R ? 1 : 0);
			}
			else {
				cerr << "R/F Error !" << endl;
				exit(-1);
			}
		}
		else if (PreT == R){
			for (int j = 0; j < CurG->NoFanin(); ++j) {
				if (CurG->Fanin(j) != PreG) {
					AddObj(ToCUTName(CurG->Fanin(j), 1), NCV);
					if(!isAndFamily){
						AddObj(ToCUTName(CurG->Fanin(j), 0), NCV);
					}
				}
			}
		}
		else if (PreT == F){
			for (int j = 0; j < CurG->NoFanin(); ++j) {
				if (CurG->Fanin(j) != PreG) {
					AddObj(ToCUTName(CurG->Fanin(j), 1), NCV);
					if(isAndFamily){
						AddObj(ToCUTName(CurG->Fanin(j), 0), NCV);
					}
				}
			}
		}
		else {
			cerr << "R/F Error !" << endl;
			exit(-1);
		}

		PreG = CurG;
		PreT = CurT;
	}
}








void AtpgObj::Dump()
{
	assert(_name.size() == _value.size());
	for (unsigned int i = 0; i < _name.size(); ++i)
	{
		cout << _name[i] << "(" << _value[i] << ") ";
	}
	cout << endl;
}
