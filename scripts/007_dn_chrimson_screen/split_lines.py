split_dict = {'S-28':{'I':'?',
					  'II':'P{w[+mC]=BJD115F05-p65ADzpUw}attP40',
					  'III':'P{w[+mC]=GMR48E11-ZpGal4DBDUw}attP2',
					  'old_name':'DN106',
					  'new_name':'P10'},
			  'S-67':{'I':'?',
					  'II':'P{w[+mC]=BJD122B03-pBPp65ADZpUw}attP40',
					  'III':'P{w[+mC]=GMR22D06-pBPZpGdbdUw}attP2',
					  'old_name':'DN066',
					  'new_name':'G25'},
					  }


cs_fly =              {'I':'w[+]',
					   'II':'P{y[+t7.7] w[+mC]=13XLexAop2-IVS-GCaMP6f-p10}su(Hw)attP5, P{y[+t7.7] w[+mC]=GMR39E01-lexA}attP40',
					   'III': 'P{20XUAS-IVS-CsChrimson.mVenus}attP2'
}

chrimson_genotypes = dict()

for key,fly1 in split_dict.items():
	chrimson_genotypes[key] = {'I':fly1['I'] + '/' + cs_fly['I'],
	                           'II': fly1['II'] + '/' + cs_fly['II'],
	                           'III': fly1['III'] + '/' + cs_fly['III']}

def expand_snum(snum):
	fly = chrimson_genotypes[snum]
	s_string = ';'.join([fly[k] for k in ['I','II','III']])
	return s_string