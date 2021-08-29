function T3Eq = GravityCompT3(Fdx,Fdy,I1,I2,I3,d1,d2,d3,g,l1,l2,l3,m1,m2,m3,th1,th2,th3,thdot1,thdot2,thdot3)
%GRAVITYCOMPT3
%    T3EQ = GRAVITYCOMPT3(FDX,FDY,I1,I2,I3,D1,D2,D3,G,L1,L2,L3,M1,M2,M3,TH1,TH2,TH3,THDOT1,THDOT2,THDOT3)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    12-Jan-2021 11:34:19

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
t162 = t153+t156;
t163 = -1.0./(t155+t144.*(t119-t135));
t167 = (t106.*t144)./(t155+t144.*(t119-t135));
t184 = t158+t161;
t213 = -t124.*((I3.*t124.*t148)./(t155+t144.*(t119-t135))+(t95.*t124.*t148)./(t155+t144.*(t119-t135))+(t96.*t124.*t148)./(t155+t144.*(t119-t135))+1.0);
t214 = t124.*((I3.*t124.*t148)./(t155+t144.*(t119-t135))+(t95.*t124.*t148)./(t155+t144.*(t119-t135))+(t96.*t124.*t148)./(t155+t144.*(t119-t135))+1.0);
t215 = -t129.*((I3.*t124.*t148)./(t155+t144.*(t119-t135))+(t105.*t124.*t148)./(t155+t144.*(t119-t135))+(t107.*t124.*t148)./(t155+t144.*(t119-t135)));
t159 = t149+t151;
t165 = t106.*t144.*t163;
t170 = t36.*t167;
t171 = t38.*t167;
t176 = t124.*t148.*t163;
t193 = t163.*t184;
t197 = (t36.*t184)./(t155+t144.*(t119-t135));
t208 = -t124.*(I3.*t167+t95.*t167+t96.*t167);
t209 = -t129.*(I3.*t167+t105.*t167+t107.*t167);
t210 = t129.*(I3.*t167+t105.*t167+t107.*t167);
t233 = t214+t215;
t160 = 1.0./t159;
t166 = I3.*t165;
t168 = t36.*t165;
t169 = t38.*t165;
t172 = t95.*t165;
t173 = t96.*t165;
t174 = t105.*t165;
t175 = t107.*t165;
t177 = I3.*t176;
t178 = t36.*t176;
t179 = t38.*t176;
t180 = t95.*t176;
t181 = t96.*t176;
t182 = t105.*t176;
t183 = t107.*t176;
t185 = t159.*t163;
t194 = I3.*t193;
t195 = t36.*t193;
t196 = t38.*t193;
t199 = t86+t197;
t218 = -t124.*((I3.*t159)./(t155+t144.*(t119-t135))+(t95.*t159)./(t155+t144.*(t119-t135))+(t96.*t159)./(t155+t144.*(t119-t135)));
t219 = -t129.*((I3.*t159)./(t155+t144.*(t119-t135))+(t105.*t159)./(t155+t144.*(t119-t135))+(t107.*t159)./(t155+t144.*(t119-t135))+1.0);
t220 = t129.*((I3.*t159)./(t155+t144.*(t119-t135))+(t105.*t159)./(t155+t144.*(t119-t135))+(t107.*t159)./(t155+t144.*(t119-t135))+1.0);
t226 = t208+t210;
t227 = -t148.*(t209+t124.*(I3.*t167+t95.*t167+t96.*t167));
t234 = t148.*t233;
t241 = -I3.*(t165+t148.*(t209+t124.*(I3.*t167+t95.*t167+t96.*t167)));
t164 = t124.*t148.*t160;
t186 = I3.*t185;
t187 = t36.*t185;
t188 = t38.*t185;
t189 = t96.*t185;
t190 = t95.*t185;
t191 = t107.*t185;
t192 = t105.*t185;
t198 = t85+t196;
t201 = m3.*t65.*t199;
t204 = m3.*t76.*t199;
t206 = t166+t172+t173;
t207 = t166+t174+t175;
t211 = t177+t180+t181-1.0;
t212 = t177+t182+t183;
t228 = I2.*t227;
t229 = t24.*t227;
t230 = t27.*t227;
t231 = t74.*t227;
t232 = t75.*t227;
t235 = I2.*t234;
t236 = t24.*t234;
t237 = t27.*t234;
t238 = t74.*t234;
t239 = t75.*t234;
t240 = t167+t227;
t244 = -m3.*t76.*(t168+t24.*t148.*(t209+t124.*(I3.*t167+t95.*t167+t96.*t167)));
t245 = -m3.*t79.*(t169+t27.*t148.*(t209+t124.*(I3.*t167+t95.*t167+t96.*t167)));
t246 = t176+t234;
t252 = t218+t220;
t253 = -1.0./(t219+t124.*((I3.*t159)./(t155+t144.*(t119-t135))+(t95.*t159)./(t155+t144.*(t119-t135))+(t96.*t159)./(t155+t144.*(t119-t135))));
t254 = -t148.*(t219+t124.*((I3.*t159)./(t155+t144.*(t119-t135))+(t95.*t159)./(t155+t144.*(t119-t135))+(t96.*t159)./(t155+t144.*(t119-t135))));
t256 = t148.*(t219+t124.*((I3.*t159)./(t155+t144.*(t119-t135))+(t95.*t159)./(t155+t144.*(t119-t135))+(t96.*t159)./(t155+t144.*(t119-t135))));
t294 = t233./(t219+t124.*((I3.*t159)./(t155+t144.*(t119-t135))+(t95.*t159)./(t155+t144.*(t119-t135))+(t96.*t159)./(t155+t144.*(t119-t135))));
t200 = m3.*t67.*t198;
t202 = -t201;
t203 = m3.*t79.*t198;
t205 = -t204;
t216 = t186+t189+t190;
t217 = t186+t191+t192-1.0;
t242 = t170+t229;
t243 = t171+t230;
t247 = I3.*t246;
t248 = t178+t236;
t249 = t179+t237;
t255 = I2.*t254;
t257 = t24.*t254;
t258 = t27.*t254;
t259 = t24.*t256;
t260 = t27.*t256;
t261 = t75.*t254;
t262 = t74.*t254;
t263 = t185+t256;
t264 = -I3.*(t254+t159./(t155+t144.*(t119-t135)));
t265 = I3.*(t254+t159./(t155+t144.*(t119-t135)));
t293 = t233.*t253;
t295 = t228+t231+t232+t241+t244+t245;
t221 = t35+t72+t73+t78+t82+t87+t194+t200+t202;
t222 = t21+t62+t89+t90+t93+t99+t104+t194+t203+t205;
t250 = m3.*t76.*t248;
t251 = m3.*t79.*t249;
t266 = t187+t259;
t267 = t188+t260;
t223 = t124.*t221;
t225 = t129.*t222;
t268 = m3.*t76.*t266;
t269 = m3.*t79.*t267;
t296 = t235+t238+t239+t247+t250+t251;
t224 = -t223;
t270 = -t268;
t271 = -t269;
t273 = -t148.*(t223-t225);
t279 = t23.*t148.*(t223-t225);
t280 = t24.*t148.*(t223-t225);
t287 = -I3.*(t148.*(t223-t225)+t184./(t155+t144.*(t119-t135)));
t290 = -m3.*t79.*(-t85+t27.*t148.*(t223-t225)+(t38.*t184)./(t155+t144.*(t119-t135)));
t298 = -1.0./(t264+t268+t269+I2.*t256+t74.*t256+t75.*t256-1.0);
t300 = (I3.*(t165+t148.*(t209+t124.*(I3.*t167+t95.*t167+t96.*t167)))+m3.*t76.*(t168+t24.*t148.*(t209+t124.*(I3.*t167+t95.*t167+t96.*t167)))+m3.*t79.*(t169+t27.*t148.*(t209+t124.*(I3.*t167+t95.*t167+t96.*t167)))+I2.*t148.*(t209+t124.*(I3.*t167+t95.*t167+t96.*t167))+t74.*t148.*(t209+t124.*(I3.*t167+t95.*t167+t96.*t167))+t75.*t148.*(t209+t124.*(I3.*t167+t95.*t167+t96.*t167)))./(t264+t268+t269+I2.*t256+t74.*t256+t75.*t256-1.0);
t305 = -1.0./(t293+t296./(t264+t268+t269+I2.*t256+t74.*t256+t75.*t256-1.0));
t272 = t224+t225;
t274 = I2.*t273;
t275 = t23.*t273;
t276 = t24.*t273;
t277 = t25.*t273;
t278 = t27.*t273;
t282 = t61+t279;
t286 = t193+t273;
t289 = t199+t280;
t297 = t255+t261+t262+t265+t270+t271+1.0;
t301 = t296.*t298;
t281 = t60+t277;
t284 = m2.*t53.*t282;
t288 = t198+t278;
t291 = m3.*t76.*t289;
t302 = t164+t301;
t304 = t294+t301;
t283 = m2.*t54.*t281;
t285 = -t284;
t292 = -t291;
t303 = 1.0./t302;
t299 = t21+t62+t89+t90+t93+t274+t283+t285+t287+t290+t292;
t306 = t298.*t299;
T3Eq = (((t223-t225)./(t219+t124.*((I3.*t159)./(t155+t144.*(t119-t135))+(t95.*t159)./(t155+t144.*(t119-t135))+(t96.*t159)./(t155+t144.*(t119-t135))))+t299./(t264+t268+t269+I2.*t256+t74.*t256+t75.*t256-1.0))./(t293+t296./(t264+t268+t269+I2.*t256+t74.*t256+t75.*t256-1.0))-t303.*(t306+t160.*t184))./(t303.*(t300-t106.*t144.*t160)+t305.*(t298.*(I3.*(t165+t148.*(t209+t124.*(I3.*t167+t95.*t167+t96.*t167)))+m3.*t76.*(t168+t24.*t148.*(t209+t124.*(I3.*t167+t95.*t167+t96.*t167)))+m3.*t79.*(t169+t27.*t148.*(t209+t124.*(I3.*t167+t95.*t167+t96.*t167)))+I2.*t148.*(t209+t124.*(I3.*t167+t95.*t167+t96.*t167))+t74.*t148.*(t209+t124.*(I3.*t167+t95.*t167+t96.*t167))+t75.*t148.*(t209+t124.*(I3.*t167+t95.*t167+t96.*t167)))+(t209+t124.*(I3.*t167+t95.*t167+t96.*t167))./(t219+t124.*((I3.*t159)./(t155+t144.*(t119-t135))+(t95.*t159)./(t155+t144.*(t119-t135))+(t96.*t159)./(t155+t144.*(t119-t135))))));
