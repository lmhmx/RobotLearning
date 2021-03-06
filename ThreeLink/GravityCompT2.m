function T2Eq = GravityCompT2(Fdx,Fdy,I1,I2,I3,d1,d2,d3,g,l1,l2,l3,m1,m2,m3,th1,th2,th3,thdot1,thdot2,thdot3)
%GRAVITYCOMPT2
%    T2EQ = GRAVITYCOMPT2(FDX,FDY,I1,I2,I3,D1,D2,D3,G,L1,L2,L3,M1,M2,M3,TH1,TH2,TH3,THDOT1,THDOT2,THDOT3)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    12-Jan-2021 11:31:03

t2 = conj(Fdx);
t3 = conj(Fdy);
t4 = conj(th1);
t5 = conj(th2);
t6 = conj(th3);
t9 = d1.^2;
t10 = d2.^2;
t11 = d3.^2;
t7 = cos(t4);
t8 = sin(t4);
t13 = t4+t5;
t12 = l1.*t7;
t14 = l1.*t8;
t15 = t7.^2;
t16 = t8.^2;
t17 = cos(t13);
t19 = sin(t13);
t21 = d1.*g.*m1.*t7;
t22 = t6+t13;
t18 = t12.*thdot1;
t20 = t14.*thdot1;
t23 = d2.*t17;
t24 = l2.*t17;
t25 = d2.*t19;
t26 = cos(t22);
t27 = l2.*t19;
t28 = sin(t22);
t29 = t17.^2;
t30 = t19.^2;
t40 = m1.*t9.*t15;
t41 = m1.*t9.*t16;
t31 = t23.*thdot2;
t32 = t24.*thdot2;
t33 = t25.*thdot2;
t34 = t27.*thdot2;
t35 = g.*m2.*t23;
t36 = d3.*t26;
t37 = l3.*t26;
t38 = d3.*t28;
t39 = l3.*t28;
t42 = t26.^2;
t43 = t28.^2;
t49 = m2.*t12.*t23;
t50 = m2.*t10.*t29;
t51 = m2.*t14.*t25;
t52 = m2.*t10.*t30;
t53 = t12+t23;
t54 = t14+t25;
t44 = t36.*thdot3;
t45 = t38.*thdot3;
t46 = t3.*t37;
t47 = t2.*t39;
t48 = g.*m3.*t36;
t56 = m3.*t12.*t36;
t57 = m3.*t11.*t42;
t58 = m3.*t14.*t38;
t59 = m3.*t11.*t43;
t60 = t18+t31;
t61 = t20+t33;
t62 = g.*m2.*t53;
t63 = m3.*t24.*t36;
t64 = m3.*t27.*t38;
t65 = t24+t36;
t66 = t24+t37;
t67 = t27+t38;
t68 = t27+t39;
t69 = m2.*t12.*t53;
t70 = m2.*t14.*t54;
t74 = m2.*t23.*t53;
t75 = m2.*t25.*t54;
t55 = -t46;
t71 = t3.*t66;
t72 = t2.*t68;
t73 = g.*m3.*t65;
t76 = t12+t65;
t77 = t12+t66;
t79 = t14+t67;
t80 = t14+t68;
t81 = m3.*t12.*t65;
t82 = m2.*t25.*t60;
t83 = m2.*t23.*t61;
t84 = m3.*t14.*t67;
t85 = t18+t32+t44;
t86 = t20+t34+t45;
t91 = m3.*t24.*t65;
t92 = m3.*t27.*t67;
t95 = m3.*t36.*t65;
t96 = m3.*t38.*t67;
t98 = I3+t56+t58;
t99 = m2.*t54.*t60;
t100 = m2.*t53.*t61;
t101 = I3+t57+t59;
t108 = I3+t63+t64;
t78 = -t71;
t87 = -t83;
t88 = t3.*t77;
t89 = g.*m3.*t76;
t90 = t2.*t80;
t94 = m3.*t12.*t76;
t97 = m3.*t14.*t79;
t102 = m3.*t24.*t76;
t103 = m3.*t27.*t79;
t104 = -t100;
t105 = m3.*t36.*t76;
t106 = 1.0./t98;
t107 = m3.*t38.*t79;
t109 = m3.*t38.*t85;
t110 = m3.*t36.*t86;
t112 = m3.*t67.*t85;
t113 = m3.*t65.*t86;
t115 = m3.*t79.*t85;
t116 = m3.*t76.*t86;
t118 = I3+t95+t96;
t123 = I2+I3+t49+t51+t81+t84;
t125 = I2+I3+t50+t52+t91+t92;
t93 = -t88;
t111 = -t110;
t114 = -t113;
t117 = -t116;
t119 = t101.*t106;
t121 = I3+t105+t107;
t122 = t106.*t108;
t124 = 1.0./t123;
t126 = I2+I3+t74+t75+t102+t103;
t128 = I1+I2+I3+t40+t41+t69+t70+t94+t97;
t120 = -t119;
t127 = t47+t48+t55+t109+t111;
t129 = 1.0./t128;
t130 = t118.*t124;
t134 = t124.*t125;
t136 = t35+t72+t73+t78+t82+t87+t112+t114;
t140 = t21+t62+t89+t90+t93+t99+t104+t115+t117;
t131 = -t130;
t132 = t106.*t127;
t135 = t121.*t129;
t137 = t126.*t129;
t141 = t124.*t136;
t146 = t129.*t140;
t133 = -t132;
t138 = -t137;
t139 = t120+t135;
t142 = -t141;
t145 = t131+t135;
t143 = t122+t138;
t147 = t134+t138;
t152 = t133+t146;
t154 = t142+t146;
t144 = 1.0./t143;
t148 = 1.0./t147;
t149 = t129.*t144;
t150 = t129.*t148;
t153 = -t144.*(t119-t135);
t155 = -t148.*(t130-t135);
t156 = t148.*(t130-t135);
t157 = -t144.*(t132-t146);
t158 = t144.*(t132-t146);
t161 = -t148.*(t141-t146);
t151 = -t150;
t164 = t153+t156;
t165 = -1.0./(t155+t144.*(t119-t135));
t168 = (t106.*t144)./(t155+t144.*(t119-t135));
t185 = t158+t161;
t214 = -t124.*((I3.*t124.*t148)./(t155+t144.*(t119-t135))+(t95.*t124.*t148)./(t155+t144.*(t119-t135))+(t96.*t124.*t148)./(t155+t144.*(t119-t135))+1.0);
t215 = t124.*((I3.*t124.*t148)./(t155+t144.*(t119-t135))+(t95.*t124.*t148)./(t155+t144.*(t119-t135))+(t96.*t124.*t148)./(t155+t144.*(t119-t135))+1.0);
t216 = -t129.*((I3.*t124.*t148)./(t155+t144.*(t119-t135))+(t105.*t124.*t148)./(t155+t144.*(t119-t135))+(t107.*t124.*t148)./(t155+t144.*(t119-t135)));
t159 = t149+t151;
t166 = t106.*t144.*t165;
t171 = t36.*t168;
t172 = t38.*t168;
t177 = t124.*t148.*t165;
t194 = t165.*t185;
t198 = (t36.*t185)./(t155+t144.*(t119-t135));
t209 = -t124.*(I3.*t168+t95.*t168+t96.*t168);
t210 = -t129.*(I3.*t168+t105.*t168+t107.*t168);
t211 = t129.*(I3.*t168+t105.*t168+t107.*t168);
t234 = t215+t216;
t160 = 1.0./t159;
t167 = I3.*t166;
t169 = t36.*t166;
t170 = t38.*t166;
t173 = t95.*t166;
t174 = t96.*t166;
t175 = t105.*t166;
t176 = t107.*t166;
t178 = I3.*t177;
t179 = t36.*t177;
t180 = t38.*t177;
t181 = t95.*t177;
t182 = t96.*t177;
t183 = t105.*t177;
t184 = t107.*t177;
t186 = t159.*t165;
t195 = I3.*t194;
t196 = t36.*t194;
t197 = t38.*t194;
t200 = t86+t198;
t219 = -t124.*((I3.*t159)./(t155+t144.*(t119-t135))+(t95.*t159)./(t155+t144.*(t119-t135))+(t96.*t159)./(t155+t144.*(t119-t135)));
t220 = -t129.*((I3.*t159)./(t155+t144.*(t119-t135))+(t105.*t159)./(t155+t144.*(t119-t135))+(t107.*t159)./(t155+t144.*(t119-t135))+1.0);
t221 = t129.*((I3.*t159)./(t155+t144.*(t119-t135))+(t105.*t159)./(t155+t144.*(t119-t135))+(t107.*t159)./(t155+t144.*(t119-t135))+1.0);
t227 = t209+t211;
t228 = -t148.*(t210+t124.*(I3.*t168+t95.*t168+t96.*t168));
t235 = t148.*t234;
t242 = -I3.*(t166+t148.*(t210+t124.*(I3.*t168+t95.*t168+t96.*t168)));
t162 = t106.*t144.*t160;
t187 = I3.*t186;
t188 = t36.*t186;
t189 = t38.*t186;
t190 = t96.*t186;
t191 = t95.*t186;
t192 = t107.*t186;
t193 = t105.*t186;
t199 = t85+t197;
t202 = m3.*t65.*t200;
t205 = m3.*t76.*t200;
t207 = t167+t173+t174;
t208 = t167+t175+t176;
t212 = t178+t181+t182-1.0;
t213 = t178+t183+t184;
t229 = I2.*t228;
t230 = t24.*t228;
t231 = t27.*t228;
t232 = t74.*t228;
t233 = t75.*t228;
t236 = I2.*t235;
t237 = t24.*t235;
t238 = t27.*t235;
t239 = t74.*t235;
t240 = t75.*t235;
t241 = t168+t228;
t245 = -m3.*t76.*(t169+t24.*t148.*(t210+t124.*(I3.*t168+t95.*t168+t96.*t168)));
t246 = -m3.*t79.*(t170+t27.*t148.*(t210+t124.*(I3.*t168+t95.*t168+t96.*t168)));
t247 = t177+t235;
t253 = t219+t221;
t254 = -1.0./(t220+t124.*((I3.*t159)./(t155+t144.*(t119-t135))+(t95.*t159)./(t155+t144.*(t119-t135))+(t96.*t159)./(t155+t144.*(t119-t135))));
t255 = -t148.*(t220+t124.*((I3.*t159)./(t155+t144.*(t119-t135))+(t95.*t159)./(t155+t144.*(t119-t135))+(t96.*t159)./(t155+t144.*(t119-t135))));
t257 = t148.*(t220+t124.*((I3.*t159)./(t155+t144.*(t119-t135))+(t95.*t159)./(t155+t144.*(t119-t135))+(t96.*t159)./(t155+t144.*(t119-t135))));
t294 = (t210+t124.*(I3.*t168+t95.*t168+t96.*t168))./(t220+t124.*((I3.*t159)./(t155+t144.*(t119-t135))+(t95.*t159)./(t155+t144.*(t119-t135))+(t96.*t159)./(t155+t144.*(t119-t135))));
t163 = -t162;
t201 = m3.*t67.*t199;
t203 = -t202;
t204 = m3.*t79.*t199;
t206 = -t205;
t217 = t187+t190+t191;
t218 = t187+t192+t193-1.0;
t243 = t171+t230;
t244 = t172+t231;
t248 = I3.*t247;
t249 = t179+t237;
t250 = t180+t238;
t256 = I2.*t255;
t258 = t24.*t255;
t259 = t27.*t255;
t260 = t24.*t257;
t261 = t27.*t257;
t262 = t75.*t255;
t263 = t74.*t255;
t264 = t186+t257;
t265 = -I3.*(t255+t159./(t155+t144.*(t119-t135)));
t266 = I3.*(t255+t159./(t155+t144.*(t119-t135)));
t295 = t254.*(t210+t124.*(I3.*t168+t95.*t168+t96.*t168));
t296 = t229+t232+t233+t242+t245+t246;
t222 = t35+t72+t73+t78+t82+t87+t195+t201+t203;
t223 = t21+t62+t89+t90+t93+t99+t104+t195+t204+t206;
t251 = m3.*t76.*t249;
t252 = m3.*t79.*t250;
t267 = t188+t260;
t268 = t189+t261;
t224 = t124.*t222;
t226 = t129.*t223;
t269 = m3.*t76.*t267;
t270 = m3.*t79.*t268;
t297 = t236+t239+t240+t248+t251+t252;
t225 = -t224;
t271 = -t269;
t272 = -t270;
t274 = -t148.*(t224-t226);
t280 = t23.*t148.*(t224-t226);
t281 = t24.*t148.*(t224-t226);
t288 = -I3.*(t148.*(t224-t226)+t185./(t155+t144.*(t119-t135)));
t291 = -m3.*t79.*(-t85+t27.*t148.*(t224-t226)+(t38.*t185)./(t155+t144.*(t119-t135)));
t299 = -1.0./(t265+t269+t270+I2.*t257+t74.*t257+t75.*t257-1.0);
t301 = (I3.*(t166+t148.*(t210+t124.*(I3.*t168+t95.*t168+t96.*t168)))+m3.*t76.*(t169+t24.*t148.*(t210+t124.*(I3.*t168+t95.*t168+t96.*t168)))+m3.*t79.*(t170+t27.*t148.*(t210+t124.*(I3.*t168+t95.*t168+t96.*t168)))+I2.*t148.*(t210+t124.*(I3.*t168+t95.*t168+t96.*t168))+t74.*t148.*(t210+t124.*(I3.*t168+t95.*t168+t96.*t168))+t75.*t148.*(t210+t124.*(I3.*t168+t95.*t168+t96.*t168)))./(t265+t269+t270+I2.*t257+t74.*t257+t75.*t257-1.0);
t273 = t225+t226;
t275 = I2.*t274;
t276 = t23.*t274;
t277 = t24.*t274;
t278 = t25.*t274;
t279 = t27.*t274;
t283 = t61+t280;
t287 = t194+t274;
t290 = t200+t281;
t298 = t256+t262+t263+t266+t271+t272+1.0;
t302 = t163+t301;
t303 = -1.0./(t162+t299.*(I3.*(t166+t148.*(t210+t124.*(I3.*t168+t95.*t168+t96.*t168)))+m3.*t76.*(t169+t24.*t148.*(t210+t124.*(I3.*t168+t95.*t168+t96.*t168)))+m3.*t79.*(t170+t27.*t148.*(t210+t124.*(I3.*t168+t95.*t168+t96.*t168)))+I2.*t148.*(t210+t124.*(I3.*t168+t95.*t168+t96.*t168))+t74.*t148.*(t210+t124.*(I3.*t168+t95.*t168+t96.*t168))+t75.*t148.*(t210+t124.*(I3.*t168+t95.*t168+t96.*t168))));
t304 = t297.*t299;
t305 = t295+t301;
t306 = -1.0./(t294+t299.*(I3.*(t166+t148.*(t210+t124.*(I3.*t168+t95.*t168+t96.*t168)))+m3.*t76.*(t169+t24.*t148.*(t210+t124.*(I3.*t168+t95.*t168+t96.*t168)))+m3.*t79.*(t170+t27.*t148.*(t210+t124.*(I3.*t168+t95.*t168+t96.*t168)))+I2.*t148.*(t210+t124.*(I3.*t168+t95.*t168+t96.*t168))+t74.*t148.*(t210+t124.*(I3.*t168+t95.*t168+t96.*t168))+t75.*t148.*(t210+t124.*(I3.*t168+t95.*t168+t96.*t168))));
t282 = t60+t278;
t285 = m2.*t53.*t283;
t289 = t199+t279;
t292 = m3.*t76.*t290;
t284 = m2.*t54.*t282;
t286 = -t285;
t293 = -t292;
t300 = t21+t62+t89+t90+t93+t275+t284+t286+t288+t291+t293;
t307 = t299.*t300;
T2Eq = (((t224-t226)./(t220+t124.*((I3.*t159)./(t155+t144.*(t119-t135))+(t95.*t159)./(t155+t144.*(t119-t135))+(t96.*t159)./(t155+t144.*(t119-t135))))+t300./(t265+t269+t270+I2.*t257+t74.*t257+t75.*t257-1.0))./(t294+t299.*(I3.*(t166+t148.*(t210+t124.*(I3.*t168+t95.*t168+t96.*t168)))+m3.*t76.*(t169+t24.*t148.*(t210+t124.*(I3.*t168+t95.*t168+t96.*t168)))+m3.*t79.*(t170+t27.*t148.*(t210+t124.*(I3.*t168+t95.*t168+t96.*t168)))+I2.*t148.*(t210+t124.*(I3.*t168+t95.*t168+t96.*t168))+t74.*t148.*(t210+t124.*(I3.*t168+t95.*t168+t96.*t168))+t75.*t148.*(t210+t124.*(I3.*t168+t95.*t168+t96.*t168))))+(t307+t160.*t185)./(t162+t299.*(I3.*(t166+t148.*(t210+t124.*(I3.*t168+t95.*t168+t96.*t168)))+m3.*t76.*(t169+t24.*t148.*(t210+t124.*(I3.*t168+t95.*t168+t96.*t168)))+m3.*t79.*(t170+t27.*t148.*(t210+t124.*(I3.*t168+t95.*t168+t96.*t168)))+I2.*t148.*(t210+t124.*(I3.*t168+t95.*t168+t96.*t168))+t74.*t148.*(t210+t124.*(I3.*t168+t95.*t168+t96.*t168))+t75.*t148.*(t210+t124.*(I3.*t168+t95.*t168+t96.*t168)))))./(t306.*(t234.*t254+t297./(t265+t269+t270+I2.*t257+t74.*t257+t75.*t257-1.0))+(t297./(t265+t269+t270+I2.*t257+t74.*t257+t75.*t257-1.0)-t124.*t148.*t160)./(t162+t299.*(I3.*(t166+t148.*(t210+t124.*(I3.*t168+t95.*t168+t96.*t168)))+m3.*t76.*(t169+t24.*t148.*(t210+t124.*(I3.*t168+t95.*t168+t96.*t168)))+m3.*t79.*(t170+t27.*t148.*(t210+t124.*(I3.*t168+t95.*t168+t96.*t168)))+I2.*t148.*(t210+t124.*(I3.*t168+t95.*t168+t96.*t168))+t74.*t148.*(t210+t124.*(I3.*t168+t95.*t168+t96.*t168))+t75.*t148.*(t210+t124.*(I3.*t168+t95.*t168+t96.*t168)))));
