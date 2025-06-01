#include "TSParser.h"
#include <iostream>
#include "christofides.h"
#include "approx2.h"
#include "Held_Karp.h"
#include <unistd.h>  // for getpid()
#include <chrono>

using namespace std;
using namespace std::chrono;

size_t get_memory_usage_kb() {
    ifstream statm("/proc/self/status");
    string line;
    while (getline(statm, line)) {
        if (line.rfind("VmRSS:", 0) == 0) {
            istringstream iss(line);
            string label;
            size_t memory_kb;
            iss >> label >> memory_kb;
            return memory_kb;  // 단위: KB
        }
    }
    return 0;
}

int main() {

    TSPParser tsp_parser("kz9976.tsp");
    tsp_parser.parse();
    tsp_parser.printInfo();

    const map<int, pair<double, double>>& node_list = tsp_parser.getNodes();
    // 상위 N개 노드 선택
    map<int, pair<double, double>> selected_nodes;
    int count = 0;
    for (const auto& [id, coord] : node_list) {
        selected_nodes[id] = coord;
        if (++count >= 100000) break;
    }

    // --- 시간 및 메모리 측정 시작 ---
    auto start = high_resolution_clock::now();
    size_t mem_before_kb = get_memory_usage_kb();

    // christofides TSP_ch(selected_nodes);s
    // approx2 TSP_apx2(selected_nodes);
    // HK_TSP tsp_hk(selected_nodes);

    // TSP_ch.execute_all();
    // TSP_apx2.execute_all();
    // tsp_hk.solve();

    auto end = high_resolution_clock::now();
    size_t mem_after_kb = get_memory_usage_kb();
    // --- 측정 종료 ---
    
    // --- 결과 출력 ---
    double duration_sec = duration_cast<duration<double>>(end - start).count();
    cout << "TSP Solve Time: " << duration_sec << " seconds" << endl;
    cout << "Memory Usage: " << (mem_after_kb - mem_before_kb) << " KB" << endl;

    //cout << "MIN COST: " << tsp_hk.min_cost() << endl;

    return 0;
}

/*
n = 22
Held-Karp TSP Solve Time: 7.4263 seconds
Memory Usage: 1892520 KB
MIN COST: 358.765
*/

/*
a280.tsp
TOTAL LENGTH: 3565
2-approximation path: 
[ 1 280 2 3 279 278 4 277 276 275 274 273 272 271 16 17 18 19 20 21 128 127 126 125 30 31 32 29 28 27 26 22 25 23 24 33 34 124 123 122 121 120 119 157 158 159 160 175 161 162 163 164 165 166 167 168 169 101 100 99 102 103 104 105 106 107 173 174 108 109 89 81 80 82 83 84 85 65 64 63 62 118 61 60 43 42 41 40 39 38 37 35 36 59 44 45 46 47 48 49 50 51 52 53 54 55 56 57 58 66 67 70 71 72 73 74 75 76 77 78 68 69 86 116 87 113 88 112 79 90 91 92 93 94 95 96 97 98 110 111 114 115 117 170 171 172 129 154 155 153 156 152 151 177 176 181 180 179 182 183 184 185 187 186 188 189 190 191 192 178 130 131 132 133 134 135 136 137 138 139 140 141 142 143 144 145 199 198 197 194 195 196 193 201 200 202 203 204 205 206 207 208 209 252 255 256 253 254 257 258 259 260 261 262 263 210 211 214 215 218 219 222 223 224 225 226 227 228 229 230 251 250 247 244 241 240 239 238 231 232 233 234 235 236 237 246 242 243 245 249 248 212 213 216 217 220 221 146 147 148 149 150 266 265 264 267 268 269 270 5 6 7 8 9 10 11 12 13 14 15 1 ]
Held-Karp TSP Solve Time: 0.00571335 seconds
Memory Usage: 640 KB
*/

/*
TOTAL LENGTH: 3575.45
TSP Solve Time: 0.00580053 seconds
Memory Usage: 768 KB
*/

/*
xql662.tsp
TOTAL LENGTH: 3552
2-approximation path: 
[ 1 8 7 33 51 42 48 9 10 11 12 13 14 15 34 16 17 3 18 19 20 21 22 4 23 24 35 25 26 27 28 29 30 31 6 32 5 2 37 44 50 54 59 69 60 61 70 62 63 64 71 65 66 72 67 68 52 45 78 79 80 81 77 82 76 75 73 74 57 58 56 49 43 86 85 84 88 87 90 97 96 95 98 93 107 132 133 134 108 109 110 111 112 113 114 115 116 117 118 130 119 120 121 122 123 124 125 126 127 138 139 128 129 140 141 131 106 94 92 91 89 83 53 55 46 40 41 47 39 36 38 154 155 165 166 159 167 160 151 157 152 164 105 101 100 103 104 135 136 137 142 143 148 156 161 162 163 170 171 181 172 173 182 174 175 176 177 180 179 169 168 190 191 192 208 193 194 195 196 197 198 199 200 201 202 210 211 218 230 240 248 249 250 251 258 264 257 256 212 213 223 231 224 232 225 242 216 183 184 215 252 254 207 206 221 228 236 222 229 237 244 246 263 262 238 189 188 187 186 205 217 220 227 235 243 260 261 274 275 276 277 278 279 280 281 282 283 284 292 285 286 287 288 289 290 301 273 272 271 259 270 296 269 268 267 294 293 302 308 315 324 314 323 336 337 334 331 335 346 325 338 347 255 245 253 247 239 241 209 214 204 185 203 158 153 265 233 295 297 309 317 327 339 354 361 360 378 377 376 359 358 388 404 405 428 427 423 413 362 363 364 365 366 380 381 382 383 367 368 369 370 371 372 373 374 385 386 392 387 389 394 402 395 403 396 412 391 375 357 390 425 426 435 424 422 421 384 401 411 419 431 432 433 414 420 434 449 448 447 446 463 462 461 445 444 443 442 441 440 439 438 437 436 467 472 486 480 489 482 490 503 511 510 509 539 512 513 514 515 516 517 518 519 520 521 522 523 524 538 525 526 527 528 529 537 530 531 532 533 534 535 536 540 545 557 541 542 546 549 547 550 548 558 544 507 508 502 543 566 567 565 564 555 554 562 573 584 572 583 561 571 582 570 560 552 553 569 581 559 568 600 587 588 601 602 627 651 653 654 655 656 657 658 659 660 661 662 551 556 591 592 593 594 595 604 605 606 596 597 598 599 608 609 610 611 612 613 618 614 615 620 625 621 626 622 634 617 589 590 616 646 647 645 644 643 642 636 641 635 607 624 631 640 650 649 639 638 637 629 630 632 633 603 619 623 628 648 652 563 574 575 576 585 577 578 586 579 580 483 491 504 492 493 473 474 475 450 451 460 452 453 454 455 456 457 458 465 466 468 469 459 477 478 481 500 499 506 498 497 505 496 484 485 487 488 501 479 464 470 471 476 495 494 379 393 398 407 415 429 397 406 399 408 416 417 418 430 400 409 410 316 326 318 328 340 341 348 349 350 355 351 352 356 353 332 307 303 304 311 321 312 322 313 333 306 291 266 305 344 345 343 342 319 329 310 320 330 298 299 300 219 226 234 144 149 145 150 102 99 146 147 178 1 ]
Held-Karp TSP Solve Time: 0.028108 seconds
Memory Usage: 1024 KB
*/

/*
TOTAL LENGTH: 3648.01
TSP Solve Time: 0.0360145 seconds
Memory Usage: 2816 KB
*/

/*
kz9976.tsp
TOTAL LENGTH: 1.45728e+06
2-approximation path: too long to contain
Held-Karp TSP Solve Time: 6.6223 seconds
Memory Usage: 5376 KB
*/

/*
TOTAL LENGTH: 1.45792e+06
TSP Solve Time: 6.97072 seconds
Memory Usage: 131864 KB
*/

/*
mona-lisa100K.tsp
TOTAL LENGTH: 8.3223e+06
2-approximation path: too long to contain
Held-Karp TSP Solve Time: 868.863 seconds
Memory Usage: 47420 KB
*/

/*
mona-lisa100K.tsp
TOTAL LENGTH: 8.3282e+06
2-approximation path: too long to contain
TSP Solve Time: 774.106 seconds
Memory Usage: 29888 KB
*/
