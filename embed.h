/*
**这个文件主要是对隐写的方式进行实现
*/
#ifndef EMBED_H
#define EMBED_H
#include <math.h>
#include <stdint.h>
#include "common/common.h"


uint32_t static const mats[] = {
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	109, 71, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	109, 79, 83, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	89, 127, 99, 69, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	95, 75, 121, 71, 109, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	71, 117, 127, 75, 89, 109, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	111, 83, 127, 97, 77, 117, 89, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	113, 111, 87, 93, 99, 73, 117, 123, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	89, 97, 115, 81, 77, 117, 87, 127, 123, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	95, 107, 109, 79, 117, 67, 121, 123, 103, 81, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	117, 71, 109, 79, 101, 115, 123, 81, 77, 95, 87, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	119, 73, 81, 125, 123, 103, 99, 127, 109, 69, 89, 107, 0, 0, 0, 0, 0, 0, 0, 0,
	87, 127, 117, 81, 97, 67, 101, 93, 105, 109, 75, 115, 123, 0, 0, 0, 0, 0, 0, 0,
	93, 107, 115, 95, 121, 81, 75, 99, 111, 85, 79, 119, 105, 65, 0, 0, 0, 0, 0, 0,
	123, 85, 79, 87, 127, 65, 115, 93, 101, 111, 73, 119, 105, 99, 91, 0, 0, 0, 0, 0,
	127, 99, 121, 111, 71, 109, 103, 117, 113, 65, 105, 87, 101, 75, 93, 123, 0, 0, 0, 0,
	89, 93, 111, 117, 103, 127, 77, 95, 85, 105, 67, 69, 113, 123, 99, 75, 119, 0, 0, 0,
	65, 99, 77, 85, 101, 91, 125, 103, 127, 111, 69, 93, 75, 95, 119, 113, 105, 115, 0, 0,
	91, 117, 77, 107, 101, 127, 115, 83, 85, 119, 105, 113, 93, 71, 111, 121, 97, 73, 81, 0,
	95, 111, 117, 83, 97, 75, 87, 127, 85, 93, 105, 115, 77, 101, 99, 89, 71, 121, 67, 123,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	247, 149, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	143, 187, 233, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	235, 141, 161, 207, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	219, 185, 151, 255, 197, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	251, 159, 217, 167, 221, 133, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	201, 143, 231, 251, 189, 169, 155, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	143, 245, 177, 253, 217, 163, 155, 197, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	233, 145, 219, 185, 231, 215, 173, 129, 243, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	139, 201, 177, 167, 213, 253, 227, 199, 185, 159, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	183, 145, 223, 199, 245, 139, 187, 157, 217, 237, 163, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	223, 145, 137, 219, 197, 243, 247, 189, 135, 181, 207, 235, 0, 0, 0, 0, 0, 0, 0, 0,
	229, 205, 237, 187, 135, 241, 183, 163, 151, 243, 213, 137, 159, 0, 0, 0, 0, 0, 0, 0,
	205, 165, 239, 211, 231, 247, 133, 227, 219, 189, 249, 185, 149, 129, 0, 0, 0, 0, 0, 0,
	131, 213, 255, 207, 227, 221, 173, 185, 197, 147, 235, 247, 217, 143, 229, 0, 0, 0, 0, 0,
	247, 139, 157, 223, 187, 147, 177, 249, 165, 153, 161, 227, 237, 255, 207, 197, 0, 0, 0, 0,
	205, 139, 239, 183, 147, 187, 249, 225, 253, 163, 173, 233, 209, 159, 255, 149, 197, 0, 0, 0,
	177, 173, 195, 137, 211, 249, 191, 135, 175, 155, 229, 215, 203, 225, 247, 237, 221, 227, 0, 0,
	159, 189, 195, 163, 255, 147, 219, 247, 231, 157, 139, 173, 185, 197, 207, 245, 193, 241, 233, 0,
	235, 179, 219, 253, 241, 131, 213, 231, 247, 223, 201, 193, 191, 249, 145, 237, 155, 165, 141, 173,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	339, 489, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	469, 441, 379, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	371, 439, 277, 479, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	413, 489, 443, 327, 357, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	509, 453, 363, 409, 425, 303, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	377, 337, 443, 487, 467, 421, 299, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	497, 349, 279, 395, 365, 427, 399, 297, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	435, 373, 395, 507, 441, 325, 279, 289, 319, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	301, 379, 509, 411, 293, 467, 455, 261, 343, 447, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	367, 289, 445, 397, 491, 279, 373, 315, 435, 473, 327, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	465, 379, 319, 275, 293, 407, 373, 427, 445, 497, 347, 417, 0, 0, 0, 0, 0, 0, 0, 0,
	473, 401, 267, 311, 359, 347, 333, 441, 405, 381, 497, 463, 269, 0, 0, 0, 0, 0, 0, 0,
	467, 283, 405, 303, 269, 337, 385, 441, 511, 361, 455, 355, 353, 311, 0, 0, 0, 0, 0, 0,
	489, 311, 259, 287, 445, 471, 419, 345, 289, 391, 405, 411, 371, 457, 331, 0, 0, 0, 0, 0,
	493, 427, 305, 309, 339, 447, 381, 335, 323, 423, 453, 457, 443, 313, 371, 353, 0, 0, 0, 0,
	271, 301, 483, 401, 369, 367, 435, 329, 319, 473, 441, 491, 325, 455, 389, 341, 317, 0, 0, 0,
	333, 311, 509, 319, 391, 441, 279, 467, 263, 487, 393, 405, 473, 303, 353, 337, 451, 365, 0, 0,
	301, 477, 361, 445, 505, 363, 375, 277, 271, 353, 337, 503, 457, 357, 287, 323, 435, 345, 497, 0,
	281, 361, 413, 287, 475, 359, 483, 351, 337, 425, 453, 423, 301, 309, 331, 499, 507, 277, 375, 471,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	519, 885, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	579, 943, 781, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	685, 663, 947, 805, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	959, 729, 679, 609, 843, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	959, 973, 793, 747, 573, 659, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	631, 559, 1023, 805, 709, 913, 979, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	607, 867, 731, 1013, 625, 973, 825, 925, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	743, 727, 851, 961, 813, 605, 527, 563, 867, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	863, 921, 943, 523, 653, 969, 563, 597, 753, 621, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	729, 747, 901, 839, 815, 935, 777, 641, 1011, 603, 973, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	581, 831, 659, 877, 781, 929, 1003, 1021, 655, 729, 983, 611, 0, 0, 0, 0, 0, 0, 0, 0,
	873, 1013, 859, 887, 579, 697, 769, 927, 679, 683, 911, 753, 733, 0, 0, 0, 0, 0, 0, 0,
	991, 767, 845, 977, 923, 609, 633, 769, 533, 829, 859, 759, 687, 657, 0, 0, 0, 0, 0, 0,
	781, 663, 731, 829, 851, 941, 601, 997, 719, 675, 947, 939, 657, 549, 647, 0, 0, 0, 0, 0,
	619, 879, 681, 601, 1015, 797, 737, 841, 839, 869, 931, 789, 767, 547, 823, 635, 0, 0, 0, 0,
	855, 567, 591, 1019, 745, 945, 769, 671, 803, 799, 925, 701, 517, 653, 885, 731, 581, 0, 0, 0,
	887, 643, 785, 611, 905, 669, 703, 1017, 575, 763, 625, 869, 731, 861, 847, 941, 933, 577, 0, 0,
	867, 991, 1021, 709, 599, 741, 933, 921, 619, 789, 957, 791, 969, 525, 591, 763, 657, 683, 829, 0,
	1009, 1003, 901, 715, 643, 803, 805, 975, 667, 619, 569, 769, 685, 767, 853, 671, 881, 907, 955, 523,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	1655, 1493, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	1859, 1481, 1119, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	1395, 1737, 1973, 1259, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	1339, 1067, 1679, 1641, 2021, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	1657, 1331, 1783, 2043, 1097, 1485, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	1611, 1141, 1849, 2001, 1511, 1359, 1245, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	1215, 1733, 1461, 2025, 1251, 1945, 1649, 1851, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	1275, 1373, 1841, 1509, 1631, 1737, 1055, 1891, 1041, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	1715, 1117, 1503, 2025, 1027, 1959, 1365, 1739, 1301, 1233, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	1101, 1127, 1145, 1157, 1195, 1747, 1885, 1527, 1325, 2033, 1935, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	1369, 1255, 1809, 1889, 1183, 1495, 1223, 1781, 2029, 1327, 1075, 1065, 0, 0, 0, 0, 0, 0, 0, 0,
	1157, 1499, 1871, 1365, 1559, 1149, 1293, 1571, 1641, 1971, 1807, 1673, 2023, 0, 0, 0, 0, 0, 0, 0,
	1929, 1533, 1135, 1359, 1547, 1723, 1529, 1107, 1273, 1879, 1709, 1141, 1897, 1161, 0, 0, 0, 0, 0, 0,
	1861, 1801, 1675, 1699, 1103, 1665, 1657, 1287, 1459, 2047, 1181, 1835, 1085, 1377, 1511, 0, 0, 0, 0, 0,
	1915, 1753, 1945, 1391, 1205, 1867, 1895, 1439, 1719, 1185, 1685, 1139, 1229, 1791, 1821, 1295, 0, 0, 0, 0,
	1193, 1951, 1469, 1737, 1047, 1227, 1989, 1717, 1735, 1643, 1857, 1965, 1405, 1575, 1907, 1173, 1299, 0, 0, 0,
	1641, 1887, 1129, 1357, 1543, 1279, 1687, 1975, 1839, 1775, 1109, 1337, 1081, 1435, 1603, 2037, 1249, 1153, 0, 0,
	1999, 1065, 1387, 1977, 1555, 1915, 1219, 1469, 1889, 1933, 1819, 1315, 1319, 1693, 1143, 1361, 1815, 1109, 1631, 0,
	1253, 1051, 1827, 1871, 1613, 1759, 2015, 1229, 1585, 1057, 1409, 1831, 1943, 1491, 1557, 1195, 1339, 1449, 1675, 1679,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	3475, 2685, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	3865, 2883, 2519, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	4019, 3383, 3029, 2397, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	2725, 3703, 3391, 2235, 2669, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	2489, 3151, 2695, 3353, 4029, 3867, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	2467, 2137, 3047, 3881, 3125, 2683, 3631, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	2739, 3163, 2137, 4031, 2967, 3413, 3749, 2301, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	3443, 2305, 3365, 2231, 2127, 3697, 3535, 4041, 2621, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	3641, 2777, 2789, 2357, 3003, 2729, 3229, 2925, 3443, 2291, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	3567, 2361, 2061, 2219, 3905, 2285, 2871, 3187, 2455, 2783, 2685, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	4043, 2615, 2385, 3911, 3267, 2871, 3667, 3037, 2905, 2921, 2129, 2299, 0, 0, 0, 0, 0, 0, 0, 0,
	2315, 2997, 3743, 2729, 3117, 2297, 2585, 3141, 3283, 3943, 3613, 3345, 4047, 0, 0, 0, 0, 0, 0, 0,
	3967, 3069, 3377, 3909, 3691, 2439, 2533, 3075, 2129, 3319, 3433, 3035, 2745, 2631, 0, 0, 0, 0, 0, 0,
	3023, 3349, 2111, 2385, 3907, 3959, 3425, 3801, 2135, 2671, 2637, 2977, 2999, 3107, 2277, 0, 0, 0, 0, 0,
	2713, 2695, 3447, 2537, 2685, 3755, 3953, 3901, 3193, 3107, 2407, 3485, 2097, 3091, 2139, 2261, 0, 0, 0, 0,
	3065, 4059, 2813, 3043, 2849, 3477, 3205, 3381, 2747, 3203, 3937, 3603, 3625, 3559, 3831, 2243, 2343, 0, 0, 0,
	3999, 3183, 2717, 2307, 2103, 3353, 2761, 2541, 2375, 2327, 3277, 2607, 3867, 3037, 2163, 2261, 3649, 2929, 0, 0,
	2543, 2415, 3867, 3709, 3161, 2369, 4087, 2205, 3785, 2515, 2133, 2913, 3941, 3371, 2605, 3269, 3385, 3025, 2323, 0,
	2939, 2775, 3663, 2413, 2573, 2205, 3821, 3513, 2699, 3379, 2479, 2663, 2367, 2517, 3027, 3201, 3177, 3281, 4069, 2069,
};

static long myholdrand = 1L;

static int myrand()
{
	return (((myholdrand = myholdrand * 214013L + 2531011L) >> 16) & 0x7fff);
}

//rotate/flip a quadrant appropriately
//static void rot(int n, int *x, int *y, int rx, int ry) {
//	if (ry == 0) {
//		if (rx == 1) {
//			*x = n - 1 - *x;
//			*y = n - 1 - *y;
//		}
//
//		//Swap x and y
//		int t = *x;
//		*x = *y;
//		*y = t;
//	}
//}
////convert (x,y) to d
//static int xy2d(int n, int x, int y) {
//	int rx, ry, s, d = 0;
//	for (s = n / 2; s > 0; s /= 2) {
//		rx = (x & s) > 0;
//		ry = (y & s) > 0;
//		d += s * s * ((3 * rx) ^ ry);
//		rot(n, &x, &y, rx, ry);
//	}
//	return d;
//}
////convert d to (x,y)
//static void d2xy(int n, int d, int *x, int *y) {
//	int rx, ry, s, t = d;
//	*x = *y = 0;
//	for (s = 1; s < n; s *= 2) {
//		rx = 1 & (t / 2);
//		ry = 1 & (t ^ rx);
//		rot(s, x, y, rx, ry);
//		*x += s * rx;
//		*y += s * ry;
//		t /= 4;
//	}
//}
//
///*宏块复杂度*/
//int macrokblock_complexity(x264_t *h)
//{
//	static uint16_t arr[3][16][16] = { 0 };
//	static int flag = 0;
//	int sum = 0;
//	static uint8_t ptr[16 * 16] = { 0 };
//	if (!flag)
//	{// 查表吧，应该要快很多
//		for (uint8_t k = 4; k <= 16; k<<=1)
//		{
//			for (uint8_t j = 0; j < k; j++)
//				for (uint8_t i = 0; i < k; i++)
//					arr[k>>3][j][i] = xy2d(k, j, i);
//		}		
//		flag = 1;
//	}
//	/*Hilbert填充曲线扫描重建宏块*/
//	uint8_t *p_dst = h->mb.pic.p_fdec[0];
//	
//	switch (h->mb.i_type)
//	{
//		case I_16x16://2
//		case I_8x8:// 1
//		case I_4x4:// 0
//#define POS(j,i) ((j>>shift_bit)*stride_j+(i>>shift_bit)*stride_i+arr[type][j%mod][i%mod])		
//			uint8_t type = h->mb.i_type;                // 0, 1,  2
//			uint8_t mod = 4 << type;                    // 4, 8,  16
//			uint8_t shift_bit = 2 + type;               // 2, 3,  4
//			uint16_t stride_j = 64 << type;              // 64,128,256
//			uint16_t stride_i = (mod)*(mod);             // 16,64, 256
//			uint16_t n = FENC_STRIDE * FENC_STRIDE - FENC_STRIDE * FENC_STRIDE / stride_i;
//			for (uint8_t j = 0; j < FENC_STRIDE; j++)
//				for (uint8_t i = 0; i < FENC_STRIDE; i++)
//					ptr[POS(j, i)] = p_dst[FDEC_STRIDE * j + i];
//			
//			for (uint8_t k = 0; k < FENC_STRIDE*FENC_STRIDE / stride_i; k++)
//				for(uint8_t i = 0; i < stride_i - 2; i++)
//					sum += abs(ptr[k*stride_i + i + 1] - ptr[k*stride_i + i]);
//			break;
//		default:
//			break;
//	}
//	
//	return sum;
//}
//
//uint8_t HamingError(uint8_t a)
//{
//	return 0;
//
//}


// Hamming code implement
//uint8_t HamingEncode(uint8_t a)
//{
//	uint8_t b = 0;
//	b = a & 0x0F;
//	b = b << 0x03;
//	if (b & 0x08) { b = b ^ 0x03; }
//	if (b & 0x10) { b = b ^ 0x05; }
//	if (b & 0x20) { b = b ^ 0x06; }
//	if (b & 0x40) { b = b ^ 0x07; }
//	return(b & 0x7F);
//}
//
//
//uint8_t HamingDecode(uint8_t a)
//{
//	uint8_t b = 0, c = 0;
//	b = a;
//	c = 0;
//	if (b & 0x40) { c = c ^ 0x07; }
//	if (b & 0x20) { c = c ^ 0x06; }
//	if (b & 0x10) { c = c ^ 0x05; }
//	if (b & 0x08) { c = c ^ 0x03; }
//	if (b & 0x04) { c = c ^ 0x04; }
//	if (b & 0x02) { c = c ^ 0x02; }
//	if (b & 0x01) { c = c ^ 0x01; }
//	switch (c) {
//		case 0:b = b >> 3; break;
//		case 1:b = b >> 3; break;
//		case 2:b = b >> 3; break;
//		case 3:b = (b ^ 0x08) >> 3; break;
//		case 4:b = b >> 3; break;
//		case 5:b = (b ^ 0x10) >> 3; break;
//		case 6:b = (b ^ 0x20) >> 3; break;
//		case 7:b = (b ^ 0x40) >> 3; break;
//	}
//	return (b & 0x0F);
//}



// 获取一个奇偶校验矩阵
static uint32_t *getMatrix(int width, int height) {
	uint32_t *cols;
	cols = (uint32_t*)malloc(width * sizeof(uint32_t));

	if (width >= 2 && width <= 20 && height >= 7 && height <= 12) { // get it from the array
		memcpy(cols, &mats[(height - 7) * 400 + (width - 1) * 20], width * sizeof(uint32_t));
	}
	else { // generate a random one
		int i, j;
		uint32_t r, mask, bop;

		if ((1 << (height - 2)) < width) {
			fprintf(stderr, "Cannot generate matrix for this payload. Choose a higher constraint height.\n");
			return NULL;
		}

		mask = (1 << (height - 2)) - 1;
		bop = (1 << (height - 1)) + 1;
		for (i = 0; i < width; i++) {
			for (j = -1; j < i;) {
				r = ((myrand() & mask) << 1) + bop;
				for (j = 0; j < i; j++) {
					if (cols[j] == r)
						break;
				}
			}
			cols[i] = r;
		}
	}
	return cols;
}

// STC 编码
static int stc_embed(uint8_t *vector, int vectorlength, uint8_t *syndrome, int syndromelength, float *pricevectorv, uint8_t *stego, int matrixheight)
{
	int height, i, k, l, index, index2, parts, m, altm, pathindex;
	uint32_t column, colmask, state;
	double totalprice;

	uint8_t *done;
	uint32_t *path, *columns[2];
	int *matrices, *widths;

	if( matrixheight > 31 )
	{
		fprintf( stderr, "Submatrix height must not exceed 31." );
		return 0;
	}

	height = 1 << matrixheight;
	colmask = height - 1;
	height = (height + 31) & (~31);

	parts = height >> 5;
	path = (uint32_t*)malloc(vectorlength * parts * sizeof(uint32_t));
	if (path == NULL)
	{
		fprintf(stderr, "Not enough memory (%u byte array could not be allocated).\n", vectorlength * parts * sizeof(uint32_t));
		return 0;
	}
	memset(path, 0, vectorlength * parts * sizeof(uint32_t));
	pathindex = 0;


	{
		int shorter, longer, worm;
		double invalpha;

		matrices = (int *)malloc(syndromelength * sizeof(int));
		widths = (int *)malloc(syndromelength * sizeof(int));

		invalpha = (double)vectorlength / syndromelength;
		if (invalpha < 1)
		{
			free(matrices);
			free(widths);
			free(path);
			fprintf(stderr, "The message cannot be longer than the cover object.\n");
			return 0;
		}
		if (invalpha < 2)
		{
			fprintf(stderr, "The relative payload is greater than 1/2. This may result in poor embedding efficiency.\n");
		}
		shorter = (int)floor(invalpha);
		longer = (int)ceil(invalpha);
		if ((columns[0] = getMatrix(shorter, matrixheight)) == NULL)
		{
			free(matrices);
			free(widths);
			free(path);
			return 0;
		}
		if ((columns[1] = getMatrix(longer, matrixheight)) == NULL)
		{
			free(columns[0]);
			free(matrices);
			free(widths);
			free(path);
			return 0;
		}
		worm = 0;
		for (i = 0; i < syndromelength; i++)
		{
			if (worm + longer <= (i + 1) * invalpha + 0.5)
			{
				matrices[i] = 1;
				widths[i] = longer;
				worm += longer;
			}
			else
			{
				matrices[i] = 0;
				widths[i] = shorter;
				worm += shorter;
			}
		}
	}

	// forward part of the Viterbi algorithm
	{
		int pathindex8 = 0;
		uint8_t mask[8] = { 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80 };
		float *prices;
		uint8_t *path8 = (uint8_t*)path;
		float *pricevector = pricevectorv; //?
		double total = 0;
		float inf;

		*(int*)&inf = 0x7F800000;	//给inf赋值无穷大

		done = (uint8_t*)malloc(height * sizeof(uint8_t));
		prices = (float*)malloc(height * sizeof(float));

		memset(done, 0, height * sizeof(uint8_t));
		for (i = 0; i < height; i++)
		{
			prices[i] = inf;
		}

		prices[0] = 0.0f;

		for (index = 0, index2 = 0; index2 < syndromelength; index2++)
		{
			float c1, c2;

			for (k = 0; k < widths[index2]; k++, index++)
			{
				column = columns[matrices[index2]][k] & colmask;

				if (vector[index] == 0)
				{
					c1 = 0.0f;
					c2 = pricevector[index];
				}
				else
				{
					c1 = pricevector[index];
					c2 = 0.0f;
				}

				total += pricevector[index]; //所有cover元素总的代价

				for (m = 0; m < height; m++)
				{
					if (!done[m])
					{
						float v1, v2, v3, v4;
						altm = m ^ column;
						v1 = prices[m];
						v2 = prices[altm];
						v3 = v1;
						v4 = v2;
						done[m] = 1;
						done[altm] = 1;

						v1 = v1 + c1;
						v2 = v2 + c2;
						v3 = v3 + c2;
						v4 = v4 + c1;

						v1 = v1 <= v2 ? v1 : v2;
						v4 = v3 <= v4 ? v3 : v4;

						prices[m] = v1;
						prices[altm] = v4;

						if (v1 == v2)
							path8[pathindex8 + (m >> 3)] = path8[pathindex8 + (m >> 3)] ^ mask[m & 7];// mask2[m%8]
						if (v4 == v3)
							path8[pathindex8 + (altm >> 3)] = path8[pathindex8 + (altm >> 3)] ^ mask[altm & 7];// mask2[altm%8]
					} //end of if(!done[m])
				} //end of state循环

				memset(done, 0, height * sizeof(uint8_t));

				pathindex += parts;
				pathindex8 += parts << 2;
			} //end of 列循环

			i = syndrome[index2] == 0 ? 0 : 1;
			for (l = 0; i < height; i += 2, l++)
			{
				prices[l] = prices[i];
			}

			if (syndromelength - index2 <= matrixheight)
				colmask >>= 1;

			for (; l < height; l++)
			{
				prices[l] = inf;
			}
		} //end of block循环

		totalprice = prices[0];	//最优路径总的代价

		free(prices);
		free(done);

		if (totalprice >= total)
		{
			free(matrices);
			free(widths);
			free(columns[0]);
			free(columns[1]);
			free(path);
			fprintf(stderr, "The syndrome is not in the range of the syndrome matrix.\n");
			return 0;
		}
	}

	// backward part of the Viterbi algorithm
	{
		pathindex -= parts;
		index--;
		index2--;
		state = 0;
		colmask = 0;

		for (; index2 >= 0; index2--) //block循环
		{
			for (k = widths[index2] - 1; k >= 0; k--, index--) //block中的列循环
			{
				if (k == widths[index2] - 1)
				{
					state = (state << 1) | syndrome[index2];
					if (syndromelength - index2 <= matrixheight)
						colmask = (colmask << 1) | 1;
				}

				if (path[pathindex + (state >> 5)] & (1 << (state & 31)))
				{
					stego[index] = 1;
					state = state ^ (columns[matrices[index2]][k] & colmask);
				}
				else {
					stego[index] = 0;
				}

				pathindex -= parts;
			}
		}
		free(path);
	}

	free(matrices);
	free(widths);
	free(columns[0]);
	free(columns[1]);

	return 1;
}


static void Hy(uint8_t code,uint8_t *m) {
	*m       = (code & 0x80) >> 7 ^ (code & 0x40) >> 6 ^ (code & 0x20) >> 5 ^ (code & 0x08) >> 3;
	*(m+1)   = (code & 0x80) >> 7 ^ (code & 0x40) >> 6 ^ (code & 0x10) >> 4 ^ (code & 0x04) >> 2;
	*(m + 2) = (code & 0x80) >> 7 ^ (code & 0x20) >> 5 ^ (code & 0x10) >> 4 ^ (code & 0x02) >> 1;
}
static uint8_t Decode(uint8_t code, uint8_t *m) {
	uint8_t index = (*m << 2) | (*(m + 1) << 1) | (*(m + 2));
	switch (index)
	{
	case 0x01:return code ^ 0x02; break;
	case 0x02:return code ^ 0x04; break;
	case 0x03:return code ^ 0x10; break;
	case 0x04:return code ^ 0x08; break;
	case 0x05:return code ^ 0x20; break;
	case 0x06:return code ^ 0x40; break;
	case 0x07:return code ^ 0x80; break;
	default:return code;
	}
}


static int wpc_embed(uint8_t *cover, uint8_t *dry, uint8_t*stego, int clen, uint8_t*message, int mlen) {
	// 模拟嵌入
	int r = 0;
	for (int i = 0; i < clen; i++) {
		if (*(dry + i)) {
			uint8_t flg = rand() % 2;// 如果是干点，满嵌的修改率应该为50%进行模拟
			if (flg) {
				*(stego + i) = *(cover + i) ^ 0x01;
				r++;
			}
			else {
				*(stego + i) = *(cover + i);
			}
		}
		else {
			*(stego + i) = *(cover + i);
		}
	}
	return r;
}



// WPC 编码
//
//void swapRowGF2(char **m, int w, int i, int j)
//{
//	for (int k = 0; k < w; k++)
//	{
//		m[i][k] ^= m[j][k];
//		m[j][k] ^= m[i][k];
//		m[i][k] ^= m[j][k];
//	}
//}
//
//void Elimination(char **Ab, int h, int w)
//{
//	int offset = 0, index = 0;
//	for (int i = 0; i < h&&i + offset < w - 1; i++)
//	{
//		if (!Ab[i][i + offset])
//		{
//			int i_max = i;
//			{
//				do
//				{
//					for (int j = i + 1; j < h; j++)
//					{
//						if (Ab[j][i + offset])
//						{
//							i_max = j;
//							break;
//						}
//					}
//				} while (i_max == i && i + (++offset) < w - 1);
//			}
//			if (i + offset < w - 1 && i_max != i)
//				swapRowGF2(Ab, w, i_max, i);
//		}
//
//		if (i + offset < w - 1 && Ab[i][i + offset])
//			for (int j = 0; j < h; j++)
//				if (j != i && Ab[j][i + offset])
//					for (int k = i + offset; k < w; k++)
//						Ab[j][k] ^= Ab[i][k];
//	}
//
//}
//
//char* GaussianEliminationGF2(char **A, int h, int w, char *b)
//{
//	// 求一个特解【Ax=b】-->Ab=0
//	char **Ab = malloc_matrix(h, w + 1);
//	for (int i = 0; i < h; i++)
//	{
//		memcpy(Ab[i], A[i], sizeof(char)*w);
//		Ab[i][w] = b[i];
//	}
//
//	Elimination(Ab, h, w + 1);
//
//	char *x = malloc_vector(w);
//	memset(x, 0, sizeof(char)*w);
//
//	int row = 0, col = 0;
//	do
//	{
//		if (Ab[row][col])
//			x[col] = Ab[row++][w];
//	} while (row < h&&++col < w);
//
//	for (int i = 0; i < h; i++)
//	{
//		char r = 0;
//		for (int j = 0; j < w; j++)
//		{
//			r ^= A[i][j] & x[j];
//		}
//		if (r != b[i])
//		{
//			free(x);
//			x = NULL;
//			// printf("try to decrease qi for embedding\n");
//			break;
//		}
//	}
//	free_matrix(Ab, h);
//	return x;
//}
//
//char **malloc_matrix(int h, int w)
//{
//	char** matrix = (char**)malloc(sizeof(char*)*h);
//	if (!matrix)
//	{
//		printf("malloc Error case 1");
//		return NULL;
//	}
//	for (int i = 0; i < h; i++)
//	{
//		matrix[i] = (char*)malloc(sizeof(char)*w);
//		if (!matrix[i])
//		{
//			printf("malloc Error case 2");
//			for (int j = 0; j < i; j++)
//				free(matrix[j]);
//			free(matrix);
//			return NULL;
//		}
//		memset(matrix[i], 0, w);
//	}
//	return matrix;
//}
//
//char** rand_matrix(int h, int w, int seed)
//{
//	char**m = malloc_matrix(h, w);
//	srand(seed);
//	for (int i = 0; i < h; i++)
//		for (int j = 0; j < w; j++)
//			m[i][j] = i == j ? 1 : i > w - 1 && j == w - 1 ? 0 : (char)rand() & 0x01;
//	//m[i][j] = i==j&&j<=w-2?1:i>w-2?0:rand()&0x01;
//	// 这样做是为了是产生的方程组始终有解
//	return m;
//}
//
//char*  rand_vector(int n)
//{
//	char *v = malloc_vector(n);
//	for (int i = 0; i < n; i++)
//		v[i] = rand() & 0x01;
//	return v;
//}
//
//char* malloc_vector(int n)
//{
//	char*m = (char*)malloc(sizeof(char)*n);
//	if (!m)
//	{
//		printf("malloc Error case 3"); return NULL;
//	}
//	memset(m, 0, n);
//	return m;
//}
//
//void free_matrix(char **m, int h)
//{
//	if (m == NULL)
//		return;
//	for (int i = 0; i < h; i++)
//		if (m[i] != NULL)
//			free(m[i]);
//	free(m);
//}
//
//void free_vector(char*m)
//{
//	if (m == NULL)
//		return;
//	free(m);
//}
//
//int sum(char*dry, int n)
//{
//	int s = 0;
//	for (int i = 0; i < n; i++)
//		s += dry[i];
//	return s;
//}
//
//char ** get_Hi_wet(char **H, char*d, int qi, int xi_len)
//{
//	int wet_num = xi_len - sum(d, xi_len);
//	char **Hi_wet = malloc_matrix(qi, wet_num);
//	int t = 0;
//	for (int j = 0; j < xi_len; j++)
//	{
//		if (!d[j]) //如果为湿点
//		{
//			for (int i = 0; i < qi; i++)
//			{
//				Hi_wet[i][t] = H[i][j];
//			}
//			t++;
//		}
//	}
//	return Hi_wet;
//}
//
//char ** get_Hi_dry(char **H, char*d, int qi, int xi_len)
//{
//	int dry_num = sum(d, xi_len);
//	// 这个地方+1是为了后面保存b
//	char **Hi_dry = malloc_matrix(qi, dry_num);
//	int t = 0;
//	for (int j = 0; j < xi_len; j++)
//	{
//		if (d[j]) //如果为干点
//		{
//			for (int i = 0; i < qi; i++)
//			{
//				Hi_dry[i][t] = H[i][j];
//			}
//			t++;
//		}
//	}
//	return Hi_dry;
//}
//
//char *get_xi_wet(char *xi, char *d, int xi_len)
//{
//	char *yi_wet = malloc_vector(xi_len - sum(d, xi_len));
//	int t = 0;
//	for (int j = 0; j < xi_len; j++)
//		if (!d[j])
//			yi_wet[t++] = xi[j];
//	return yi_wet;
//}
//
//char *Matrix_multi(char **Hi, int h, int w, char*xi)
//{
//	char *yi = malloc_vector(h);
//	for (int i = 0; i < h; i++)
//		for (int j = 0; j < w; j++)
//			yi[i] ^= Hi[i][j] & xi[j];
//	return yi;
//}
//
//int read_qbit(char *bit, int q)
//{
//	int q_end = 0;
//	for (int i = 0; i < q; i++)
//		q_end += bit[i] << (q - i - 1);
//	return q_end;
//}
//
//char* wpc_embed(char*cover, char *dry_point, int  n, char *bit, int m, float r2, int Kavg, int seed)
//{
//	// 分段数 r2 干湿比例，Kavg 每组干点数量
//	int beta = (int)ceil(n*r2 / Kavg);
//	// 协议头长度
//	char h = ceil(log(r2*n / beta) / log(2));
//	// 嵌入消息与协议头总长度,约等于干点数所需要的尺寸
//	int q = m + beta * h;
//
//	// 将cover分成beta段；前t2组的cover为t1+1比特，后边的组的cover 为t1比特
//	int t1 = floor(n / beta);
//	int t2 = n % beta;
//	int dry_num = sum(dry_point, n);
//
//	// H
//	char ** H = rand_matrix(t1 + 1, t1 + 1, seed);
//	// stego
//	char *stego = malloc_vector(n);
//	// message,如果是只有1段，将h比特放在消息前面
//	char *p_bit = malloc_vector(h*beta + m);
//
//	if (NULL == H || NULL == stego || NULL == p_bit)
//	{
//		if (NULL != H)
//			free_matrix(H, t1 + 1);
//		if (NULL != stego)
//			free_vector(stego);
//		if (p_bit != NULL)
//			free_vector(p_bit);
//		return NULL;
//	}
//
//	char *p_stego = stego;
//	char *p_cover = cover;
//	char *p_dry = dry_point;
//	char *p_bit_temp = p_bit;
//	memcpy(p_bit, bit, sizeof(char)*m);
//
//	// 待嵌bit数
//	int m_less = m;
//
//	char **Hi_wet = NULL;
//	char **Hi_dry = NULL;
//	char * xi_wet = NULL;
//	char * mi_wet = NULL;
//	// 每一次的输入
//	char *xi = NULL;
//	// 每一次输入对应的干湿点
//	char *d = NULL;
//
//	int ki = 0;
//	int qi = 0;
//	int xi_len = 0;
//
//
//	for (int ii = 0; ii < beta - 1; ii++)
//	{
//		xi_len = (ii < t2 ? t1 + 1 : t1);
//		// 本次输入
//		xi = p_cover;
//		d = p_dry;
//		// 下一次的输入
//		p_cover += xi_len;
//		p_dry += xi_len;
//
//		// 输入对应的干点数量
//		ki = sum(d, xi_len);
//		qi = ceil(ki*(q + 10) / dry_num);
//
//		if (qi > m_less)
//			qi = m_less;
//		// Hi ki*qi
//		// qi*(xi_len-ki)
//		Hi_wet = get_Hi_wet(H, d, qi, xi_len);
//		// qi*ki+1 Hi_dry[:,ki]是b
//		Hi_dry = get_Hi_dry(H, d, qi, xi_len);
//		// 1*(xi_len-ki)
//		xi_wet = get_xi_wet(xi, d, xi_len);
//
//		while (1)
//		{//如果嵌入qi bit失败，则尝试少嵌入1bit,直到成功
//			// 如果嵌入的信息bit大于剩余的信息bit
//			// 1*qi
//			mi_wet = Matrix_multi(Hi_wet, qi, xi_len - ki, xi_wet);
//			char *mi_dry = mi_wet;
//
//			for (int i = 0; i < qi; i++)
//				mi_dry[i] = mi_wet[i] ^ p_bit[i];
//
//			char *x = GaussianEliminationGF2(Hi_dry, qi, ki, mi_dry);
//			if (!x)
//			{
//				qi -= 1; //尝试减少1bit
//				free_vector(mi_wet);
//			}
//			else { // 成功
//				p_bit += qi; //
//				m_less -= qi; // 剩余的信息减qi比特
//				q -= qi;
//				dry_num -= ki;
//				for (int k = 0, t = 0; k < xi_len; k++)
//					*p_stego++ = d[k] ? x[t++] : xi[k];
//
//				for (int k = 0; k < h; k++)
//					*(p_bit + m_less++) = (qi >> (h - k - 1)) & 0x01;
//
//				free_matrix(Hi_wet, qi);
//				free_matrix(Hi_dry, qi);
//				free_vector(mi_wet);
//				free_vector(xi_wet);
//				free_vector(x);
//				break;
//			}
//		}
//	}
//
//	// 处理最后一个分组,将h bit 插入在前面
//	// 将剩余的信息比特后移h比特，以便存放最后的头信息
//	memcpy(p_bit + h, p_bit, m_less);
//
//	for (int i = 0; i < h; i++, m_less++)
//	{
//		*(p_bit + i) = (q >> (h - i - 1)) & 0x01;
//	}
//	//m_less 应该是等于q的
//	// 最后一部分xi和d
//	xi = p_cover;
//	d = p_dry;
//	xi_len = n - (xi - cover); //剩余个数
//	p_cover += xi_len;
//	p_dry += xi_len;
//
//	ki = sum(d, xi_len);
//	// q*(xi_len-ki)
//	Hi_wet = get_Hi_wet(H, d, q, xi_len);
//	// q*ki+1 1是b
//	Hi_dry = get_Hi_dry(H, d, q, xi_len);
//	// 1*(xi_len-ki)
//	xi_wet = get_xi_wet(xi, d, xi_len);
//	// 1*q
//	mi_wet = Matrix_multi(Hi_wet, q, xi_len - ki, xi_wet);
//
//	char *mi_dry = mi_wet;
//	for (int i = 0; i < q; i++)
//		mi_dry[i] = mi_wet[i] ^ p_bit[i];
//	p_bit += q;
//
//	char *x = GaussianEliminationGF2(Hi_dry, q, ki, mi_dry);
//	if (x)
//	{
//		for (int i = 0, t = 0; i < xi_len; i++)
//			*p_stego++ = d[i] ? x[t++] : xi[i];
//	}
//	else {
//		free_vector(stego);
//		stego = NULL;
//	}
//	free_matrix(H, t1 + 1);
//	free_matrix(Hi_dry, q);
//	free_matrix(Hi_wet, q);
//	free_vector(xi_wet);
//	free_vector(mi_wet);
//	free_vector(p_bit_temp);
//	free_vector(x);
//	return stego;
//}



// 互信息相关的函数
//typedef unsigned int uint;
//#define LOG_BASE 2.0
//#define CALLOC_FUNC(a,b) calloc(a,b)
//#define FREE_FUNC(a) free(a)
//
//typedef struct jpState
//{
//	double *jointProbabilityVector;
//	int numJointStates;
//	double *firstProbabilityVector;
//	int numFirstStates;
//	double *secondProbabilityVector;
//	int numSecondStates;
//}jpS;
//
//void* checkedCalloc(size_t vectorLength, size_t sizeOfType) {
//	void *allocated = CALLOC_FUNC(vectorLength, sizeOfType);
//	if (allocated == NULL) {
//		fprintf(stderr, "Error: %s\nAttempted to allocate %lu length of size %lu\n", strerror(errno), vectorLength, sizeOfType);
//		exit(0);
//	}
//	return allocated;
//}
//
//void freeJointProbabilityState(jpS state) {
//	FREE_FUNC(state.firstProbabilityVector);
//	state.firstProbabilityVector = NULL;
//	FREE_FUNC(state.secondProbabilityVector);
//	state.secondProbabilityVector = NULL;
//	FREE_FUNC(state.jointProbabilityVector);
//	state.jointProbabilityVector = NULL;
//}
//
//void arrayInt2Uint(int *inputVector, uint *outputVector, int vectorLength) {
//	int minVal = 0;
//	int i;
//
//	if (vectorLength > 0) {
//
//		minVal = inputVector[0];
//
//		for (i = 0; i < vectorLength; i++) {
//			if (inputVector[i] < minVal) {
//				minVal = inputVector[i];
//			}
//		}
//
//		for (i = 0; i < vectorLength; i++) {
//			outputVector[i] = inputVector[i] - minVal;
//		}
//	}
//}
//
//int maxState(uint *vector, int vectorLength) {
//	int i, max;
//	max = 0;
//	for (i = 0; i < vectorLength; i++) {
//		if (vector[i] > max) {
//			max = vector[i];
//		}
//	}
//	return max + 1;
//}
//
//jpS calculateJointProbability(uint *firstVector, uint *secondVector, int vectorLength) {
//	int *firstStateCounts;
//	int *secondStateCounts;
//	int *jointStateCounts;
//	double *firstStateProbs;
//	double *secondStateProbs;
//	double *jointStateProbs;
//	int firstNumStates;
//	int secondNumStates;
//	int jointNumStates;
//	int i;
//	double length = vectorLength;
//	jpS state;
//
//	firstNumStates = maxState(firstVector, vectorLength);
//	secondNumStates = maxState(secondVector, vectorLength);
//	jointNumStates = firstNumStates * secondNumStates;
//
//	firstStateCounts = (int *)checkedCalloc(firstNumStates, sizeof(int));
//	secondStateCounts = (int *)checkedCalloc(secondNumStates, sizeof(int));
//	jointStateCounts = (int *)checkedCalloc(jointNumStates, sizeof(int));
//
//	firstStateProbs = (double *)checkedCalloc(firstNumStates, sizeof(double));
//	secondStateProbs = (double *)checkedCalloc(secondNumStates, sizeof(double));
//	jointStateProbs = (double *)checkedCalloc(jointNumStates, sizeof(double));
//
//	/* Optimised for number of FP operations now O(states) instead of O(vectorLength) */
//	for (i = 0; i < vectorLength; i++) {
//		firstStateCounts[firstVector[i]] += 1;
//		secondStateCounts[secondVector[i]] += 1;
//		jointStateCounts[secondVector[i] * firstNumStates + firstVector[i]] += 1;
//	}
//
//	for (i = 0; i < firstNumStates; i++) {
//		firstStateProbs[i] = firstStateCounts[i] / length;
//	}
//
//	for (i = 0; i < secondNumStates; i++) {
//		secondStateProbs[i] = secondStateCounts[i] / length;
//	}
//
//	for (i = 0; i < jointNumStates; i++) {
//		jointStateProbs[i] = jointStateCounts[i] / length;
//	}
//
//	FREE_FUNC(firstStateCounts);
//	FREE_FUNC(secondStateCounts);
//	FREE_FUNC(jointStateCounts);
//
//	firstStateCounts = NULL;
//	secondStateCounts = NULL;
//	jointStateCounts = NULL;
//
//	state.jointProbabilityVector = jointStateProbs;
//	state.numJointStates = jointNumStates;
//	state.firstProbabilityVector = firstStateProbs;
//	state.numFirstStates = firstNumStates;
//	state.secondProbabilityVector = secondStateProbs;
//	state.numSecondStates = secondNumStates;
//
//	return state;
//}
//
//double mi(jpS state) {
//	double mutualInformation = 0.0;
//	int firstIndex, secondIndex;
//	int i;
//	for (i = 0; i < state.numJointStates; i++) {
//		firstIndex = i % state.numFirstStates;
//		secondIndex = i / state.numFirstStates;
//		if ((state.jointProbabilityVector[i] > 0) && (state.firstProbabilityVector[firstIndex] > 0) && (state.secondProbabilityVector[secondIndex] > 0)) {
//			mutualInformation += state.jointProbabilityVector[i] * log(state.jointProbabilityVector[i] / state.firstProbabilityVector[firstIndex] / state.secondProbabilityVector[secondIndex]);
//		}
//	}
//	mutualInformation /= log(LOG_BASE);
//	return mutualInformation;
//}
//
//double calcNormalizedMutualInformation(uint *dataVector, uint *targetVector, int vectorLength) {
//	jpS state = calculateJointProbability(dataVector, targetVector, vectorLength);
//	int i = 0;
//	double HX = 0.0, HY = 0.0;
//	double tempValue = 0.0;
//	double normalizedMutualInformation;
//	double mutualInformation = mi(state);
//	for (i = 0; i < state.numFirstStates; i++) {
//		tempValue = state.firstProbabilityVector[i];
//		if (tempValue > 0) {
//			HX -= tempValue * log(tempValue);
//		}
//	}
//	HX /= log(LOG_BASE);
//	for (i = 0; i < state.numSecondStates; i++) {
//		tempValue = state.secondProbabilityVector[i];
//		if (tempValue > 0) {
//			HY -= tempValue * log(tempValue);
//		}
//	}
//	HY /= log(LOG_BASE);
//	normalizedMutualInformation = 2 * mutualInformation / (HX + HY);
//	freeJointProbabilityState(state);
//	return normalizedMutualInformation;
//}/*calcNormalizedMutualInformation(uint *, uint *, int ) add by hqtang 2020.04.09*/
//


#endif
