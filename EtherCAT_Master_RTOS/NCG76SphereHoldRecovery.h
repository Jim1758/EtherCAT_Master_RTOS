#pragma once
// Included after Cycle's implementation. All state is persistent and bounded;
// the immutable history belongs to the current RT parent, never old authority.
namespace NCG76SphereCycle
{
inline NCG76SphereRetreat::Stage RecoveryNominalStage(Phase phase) noexcept
{
    return phase==Phase::ContourFinish?NCG76SphereRetreat::Stage::ContourFinish:
        phase==Phase::ContourNominal?NCG76SphereRetreat::Stage::ContourNominal:
        static_cast<NCG76SphereRetreat::Stage>(static_cast<unsigned>(phase)-static_cast<unsigned>(Phase::InitialDepth));
}
inline bool Cycle::RecoveryTotal(const NCG76SphereCycle::Geometry& g,double& total) const noexcept
{
    NCG76SphereRetreat::Span span{};
    if(g.kind==NCG76SphereRuntimeGeometry::Kind::OrbitContour)return NCG76SphereRetreat::ContourTotal(g,total);
    const bool sphere=g.kind==NCG76SphereRuntimeGeometry::Kind::Sphere;
    if(!NCG76SphereRetreat::GetSpan(g,sphere?NCG76SphereRetreat::Stage::AdvanceCenter:
        NCG76SphereRetreat::Stage::InitialDepth,sphere?g.circleLayerCount-1U:0U,span))return false;
    total=span.end;return std::isfinite(total)&&total>=0.0;
}
inline const NCG76SphereCycle::Geometry& Cycle::RecoveryGeometry(unsigned ordinal) const noexcept
{return ordinal==recovery_.historyCount?request_.geometry:recovery_.history[ordinal];}
// A stopped interior scalar can round onto a canonical station endpoint.
// Exact coordinate aliases may retain that endpoint station. A distinct point
// may use only a proved conservative station floor; the saved leg, scalar
// progress and physical command remain unchanged in either case.
inline bool ResolveStoppedRecoveryStation(const Geometry& geometry,NCG76SphereRetreat::Stage stage,
    unsigned layer,double progress,double length,double speed,const Point8& anchor,double& station) noexcept
{
    if(!Detail::Positive(length)||!std::isfinite(progress)||progress<0.0||progress>length||
       !std::isfinite(speed)||speed!=0.0||!Detail::Finite(anchor))return false;
    const double fraction=progress/length;
    if(!(fraction>0.0&&fraction<1.0))return false;
    NCG76SphereRetreat::Span span{};
    if(!NCG76SphereRetreat::GetSpan(geometry,stage,layer,span))return false;
    const double rounded=span.start+(span.end-span.start)*fraction;
    const bool atStart=rounded==span.start,atEnd=rounded==span.end;
    if(!atStart&&!atEnd)return false;
    const double residual=atStart?progress:length-progress;
    const double endpointStation=atStart?span.start:span.end;
    const double interiorStation=std::nextafter(endpointStation,atStart?
        std::numeric_limits<double>::infinity():-std::numeric_limits<double>::infinity());
    const double stationUlp=std::fabs(interiorStation-endpointStation);
    const double budget=geometry.arithmeticErrorBoundMm;
    if(!Detail::Positive(budget)||budget>NCG76SphereRuntimeGeometry::MaximumArithmeticErrorMm||
       !Detail::Positive(stationUlp)||!std::isfinite(residual)||residual<0.0||
       residual>(std::min)(budget,stationUlp))return false;
    Point8 endpoint{},interior{},center{};
    if(!NCG76SphereRetreat::Evaluate(geometry,span,atStart?0.0:1.0,endpoint,center)||
       !NCG76SphereRetreat::Evaluate(geometry,span,fraction,interior,center))return false;
    if(std::memcmp(endpoint.data(),interior.data(),sizeof(Point8))==0&&
       std::memcmp(endpoint.data(),anchor.data(),sizeof(Point8))==0)
    {station=endpointStation;return true;}
    // There is no preceding canonical distance at an aliased start. Only an
    // end can lend its strictly interior floor to a distinct physical point.
    if(!atEnd||!(interiorStation>span.start&&interiorStation<span.end))return false;
    const double infinity=std::numeric_limits<double>::infinity();
    const double widthUpper=std::nextafter(span.end-span.start,infinity);
    const double ratioUpper=std::nextafter(residual/length,infinity);
    const double mappedResidualUpper=std::nextafter(ratioUpper*widthUpper,infinity);
    if(!Detail::Positive(widthUpper)||!Detail::Positive(ratioUpper)||
       !Detail::Positive(mappedResidualUpper)||mappedResidualUpper>stationUlp)return false;
    Point8 projected{};
    if(!NCG76SphereRetreat::Evaluate(geometry,span,
        NCG76SphereRetreat::Fraction(span,interiorStation),projected,center))return false;
    // Scalar ordering is fenced by a station ULP above. Native coordinate
    // reconstruction has its own already-admitted arithmetic error budget;
    // a station ULP is not a bound on three separately rounded XYZ axes.
    const double projectionBudget=budget;
    double projection=0.0;
    for(const Point8* candidate:{&endpoint,&interior,&projected}){
        if(std::memcmp(candidate->data()+3U,anchor.data()+3U,5U*sizeof(double))!=0)return false;
        Point3 error{};for(unsigned axis=0U;axis<3U;++axis)error[axis]=(*candidate)[axis]-anchor[axis];
        if(!Detail::Finite(error))return false;
        const double norm=Detail::Norm(error);
        const double upper=norm==0.0?0.0:std::nextafter(std::nextafter(norm,infinity),infinity);
        if(!std::isfinite(upper)||upper>projectionBudget)return false;
        projection=(std::max)(projection,upper);
    }
    const double combinedUpper=std::nextafter(projection+mappedResidualUpper,infinity);
    if(!std::isfinite(combinedUpper)||combinedUpper>budget)return false;
    station=interiorStation;return true;
}
inline bool Cycle::RecoveryStation(Phase phase,unsigned layer,const Leg& leg,double& station) const noexcept
{
    if(request_.geometry.kind==NCG76SphereRuntimeGeometry::Kind::Line&&request_.geometry.travelMm==0.0)
    {station=0.0;return true;}
    if(!Detail::IsNominal(phase)||!Detail::Positive(leg.length))return false;
    const auto stage=RecoveryNominalStage(phase);
    if(NCG76SphereRetreat::Station(request_.geometry,stage,layer,leg.position/leg.length,station))return true;
    if(feedback_.gapHistoryActive)return false;
    const Point8& anchor=feedback_.flushCheckpointSaved?savedAnchor_:output_.commandMCS;
    return ResolveStoppedRecoveryStation(request_.geometry,stage,layer,
        leg.position,leg.length,leg.speed,anchor,station);
}
inline bool Cycle::ConfigureHoldRecovery(const NCHoldRecovery::Config& config,std::uint32_t declaredCount,
    const NCG76SphereCycle::Geometry* completedOldestFirst,std::uint32_t completedCount) noexcept
{
    if(declaredCount>15U||completedCount>15U||(completedCount&&!completedOldestFirst))return false;
    if(completedCount&&NCG76SphereRuntimeGeometry::Detail::Overlap(this,sizeof(*this),
        completedOldestFirst,sizeof(*completedOldestFirst)*completedCount))return false;
    for(unsigned i=0U;i<completedCount;++i)
        if(!NCG76SphereRuntimeGeometry::Valid(completedOldestFirst[i]))return false;
    if(!ConfigureHoldRecoveryAdmitted(config,declaredCount,completedOldestFirst,completedCount))return false;
    // Standalone admission has only its supplied travel envelope. Production
    // Motion uses the private admitted entry and supplies independent installed
    // inspection bounds through its separate private setter.
    (void)ConfigureInspectionTravel(request_.travelMin,request_.travelMax,request_.travelMask&7U);
    return true;
}
inline bool Cycle::ConfigureHoldRecoveryAdmitted(const NCHoldRecovery::Config& config,std::uint32_t declaredCount,
    const NCG76SphereCycle::Geometry* completedOldestFirst,std::uint32_t completedCount) noexcept
{
    if(!initialized_||!output_.active||recovery_.enabled||feedback_.holdActive||declaredCount>15U||
       completedCount>15U||(completedCount&&!completedOldestFirst)||feedback_.phase!=Phase::Settle||leg_.position!=0.0||leg_.speed!=0.0)
        return false;
    if(!NCHoldRecovery::Valid(config))return false;
    if(completedCount&&NCG76SphereRuntimeGeometry::Detail::Overlap(this,sizeof(*this),
        completedOldestFirst,sizeof(*completedOldestFirst)*completedCount))return false;
    for(unsigned i=0U;i<completedCount;++i){
        const auto& g=completedOldestFirst[i];double total=0.0;
        if(!g.valid||g.version!=NCG76SphereRuntimeGeometry::Version||!RecoveryTotal(g,total)||
           !SameNCTranslationSnapshot(g.frame.translation,request_.geometry.frame.translation))return false;
        const auto& next=i+1U==completedCount?request_.geometry:completedOldestFirst[i+1U];
        if(std::memcmp(g.endMCS.data(),next.startMCS.data(),sizeof(Point8))!=0)return false;
        for(unsigned a=0U;a<8U;++a){
            if(a>=3U&&(g.startMCS[a]!=request_.geometry.startMCS[a]||g.endMCS[a]!=request_.geometry.startMCS[a]))return false;
            if((request_.travelMask&(1U<<a))&&(g.boundsMinMCS[a]<request_.travelMin[a]||g.boundsMaxMCS[a]>request_.travelMax[a]))return false;
        }
    }
    recovery_.config=config;recovery_.history=completedOldestFirst;recovery_.historyCount=completedCount;
    recovery_.enabled=true;feedback_.holdRecoveryEnabled=true;feedback_.holdRecoveryPhase=1U;
    feedback_.holdDeclaredCount=declaredCount;feedback_.holdAvailableCount=completedCount;
    feedback_.holdEffectiveCount=(std::min)(declaredCount,completedCount);
    recovery_.targetOrdinal=completedCount-feedback_.holdEffectiveCount;
    feedback_.inspectionTraceValid=true;
    recovery_.retreatAnchor=RecoveryGeometry(recovery_.targetOrdinal).startMCS;
    return true;
}
inline bool Cycle::ConfigureInspectionTravel(const Point8& minimum,const Point8& maximum,std::uint32_t mask) noexcept
{
    if(!recovery_.enabled||recovery_.active)return false;
    recovery_.inspectionTravelReady=false;
    if(mask!=7U)return false;
    for(unsigned a=0U;a<3U;++a)
        if(!std::isfinite(minimum[a])||!std::isfinite(maximum[a])||minimum[a]>=maximum[a]||
           !std::isfinite(maximum[a]-minimum[a]))return false;
    recovery_.inspectionMinimum=minimum;recovery_.inspectionMaximum=maximum;
    recovery_.inspectionTravelReady=true;return true;
}
inline bool Cycle::InspectionTravelAvailable() const noexcept
{
    if(!recovery_.enabled||!recovery_.inspectionTravelReady)return false;
    for(unsigned a=0U;a<3U;++a)
        if(!std::isfinite(output_.commandMCS[a])||output_.commandMCS[a]<recovery_.inspectionMinimum[a]||
           output_.commandMCS[a]>recovery_.inspectionMaximum[a])return false;
    return true;
}
inline bool Cycle::IsInspectionTravelActive() const noexcept
{
    if(!recovery_.active||!recovery_.inspectionTravelReady)return false;
    using S=RecoveryStage;const auto stage=recovery_.stage;
    return stage==S::Jog||stage==S::JogStopping||stage==S::JogSettle||stage==S::JogReturn||
        (stage==S::Held&&recovery_.inspectionCount>0U)||
        ((stage==S::PauseBraking||stage==S::PauseSettle||stage==S::Paused)&&recovery_.pausedStage==S::JogReturn);
}
inline void Cycle::RefreshRecoveryFeedback() noexcept
{
    feedback_.holdRecoveryEnabled=recovery_.enabled;
    feedback_.holdRecoveryPaused=IsHoldRecoveryPaused();
    feedback_.inspectionTraceCount=recovery_.inspectionCount;
    if(output_.faulted||feedback_.fault!=Fault::None){feedback_.holdRecoveryPhase=10U;return;}
    if(!recovery_.active){feedback_.holdRecoveryPhase=1U;feedback_.inspectionJogActive=false;return;}
    if(recovery_.captured){
        feedback_.nominalProgressMm=recovery_.sourceFeedback.nominalProgressMm;
        feedback_.nominalLengthMm=recovery_.sourceFeedback.nominalLengthMm;
        feedback_.previousParentHistoryActive=recovery_.sourceFeedback.previousParentHistoryActive;
        feedback_.previousParentProgressMm=recovery_.sourceFeedback.previousParentProgressMm;
        feedback_.previousParentLengthMm=recovery_.sourceFeedback.previousParentLengthMm;
    }else if(!feedback_.flushCheckpointSaved&&!feedback_.gapHistoryActive){
        feedback_.nominalProgressMm=leg_.position;feedback_.nominalLengthMm=leg_.length;
    }
    feedback_.holdActive=true;feedback_.gapApplied=false;
    using S=RecoveryStage;
    const auto stage=recovery_.stage;
    feedback_.inspectionJogActive=stage==S::Jog||stage==S::JogStopping||stage==S::JogSettle;
    feedback_.holdRecoveryPhase=stage==S::Capture?2U:
        stage==S::Held||stage==S::Paused||stage==S::PauseSettle?4U:
        stage==S::Jog||stage==S::JogStopping||stage==S::JogSettle?5U:
        stage==S::JogReturn?6U:stage==S::Forward?7U:
        stage==S::BridgeInLines||stage==S::BridgeInPath||stage==S::Restore?8U:
        stage==S::PauseBraking?2U:3U;
}
inline bool Cycle::StartRecoveryLine(const Point8& end,bool returning,Phase phase) noexcept
{
    if(!InsideTravel(end)){Fail(Fault::TravelLimit);return false;}
    const auto& c=recovery_.config;
    const bool jog=recovery_.stage==RecoveryStage::JogReturn;
    const double feed=jog?c.jogRecoveryFeedMmMin:returning?c.resumeFeedMmMin:c.retreatFeedMmMin;
    const double acc=jog?c.jogAccelSeconds:returning?c.resumeAccelSeconds:c.retreatAccelSeconds;
    const double dec=jog?c.jogDecelSeconds:returning?c.resumeDecelSeconds:c.retreatDecelSeconds;
    Leg next{};next.start=output_.commandMCS;next.end=end;
    for(unsigned a=0U;a<3U;++a)next.direction[a]=end[a]-next.start[a];
    for(unsigned a=3U;a<8U;++a)if(end[a]!=request_.geometry.startMCS[a]||next.start[a]!=end[a]){Fail(Fault::Numeric);return false;}
    next.length=Detail::Norm(next.direction);
    if(next.length>0.0)for(double& x:next.direction)x/=next.length;
    next.speedLimit=(std::min)(feed/60.0,request_.velocityCapMmS);
    next.acceleration=(std::min)(request_.accelerationMmS2,next.speedLimit/acc);
    next.deceleration=(std::min)(request_.decelerationMmS2,next.speedLimit/dec);
    if(!Detail::Positive(next.speedLimit)||!StartProfile(next)){Fail(Fault::Numeric);return false;}
    leg_=next;recovery_.evaluationGeometry=nullptr;feedback_.phase=phase;
    output_.velocityNative.fill(0.0);output_.accelerationNative.fill(0.0);RefreshOutput();return true;
}
inline bool Cycle::StartRecoveryPath(const NCG76SphereCycle::Geometry& g,double fromStation,double toStation,bool returning) noexcept
{
    using namespace NCG76SphereRetreat;
    if(fromStation==toStation)return false;
    const bool reverse=toStation<fromStation;
    Span span{};if(!Locate(g,fromStation,reverse,span)){Fail(Fault::Numeric);return false;}
    const double finish=reverse?(std::max)(toStation,span.start):(std::min)(toStation,span.end);
    const double from=Fraction(span,fromStation),to=Fraction(span,finish);
    Point8 end{},center{};
    if(!Evaluate(g,span,to,end,center)||!(reverse?finish<fromStation:finish>fromStation)){Fail(Fault::Numeric);return false;}
    if(recovery_.stage==RecoveryStage::Forward&&recovery_.ordinal==recovery_.historyCount&&finish==recovery_.nominalStation)
        end=recovery_.nominalAnchor;
    if(recovery_.stage==RecoveryStage::BridgeOutPath&&finish==recovery_.bridgeEndStation)end=recovery_.nominalAnchor;
    if(recovery_.stage==RecoveryStage::BridgeInPath&&finish==recovery_.bridgeStartStation)end=recovery_.processPoint;
    recovery_.legStartStation=fromStation;recovery_.legEndStation=finish;
    const bool arc=g.kind==NCG76SphereRuntimeGeometry::Kind::Arc||span.stage==Stage::Orbit;
    if(!arc){const bool ok=StartRecoveryLine(end,returning);recovery_.evaluationGeometry=&g;return ok;}
    NCG76SphereRuntimeGeometry::Layer layer{};
    if(g.kind!=NCG76SphereRuntimeGeometry::Kind::Arc&&!NCG76SphereRuntimeGeometry::LayerAt(g,span.layer,layer)){Fail(Fault::Numeric);return false;}
    const double angle=g.kind==NCG76SphereRuntimeGeometry::Kind::Arc?g.arcSweepRad:NCG76SphereRuntimeGeometry::TwoPi;
    const double radius=g.kind==NCG76SphereRuntimeGeometry::Kind::Arc?g.radiusMm:layer.radiusMm;
    const auto& c=recovery_.config;
    const double feed=returning?c.resumeFeedMmMin:c.retreatFeedMmMin;
    const double acc=returning?c.resumeAccelSeconds:c.retreatAccelSeconds;
    const double dec=returning?c.resumeDecelSeconds:c.retreatDecelSeconds;
    Leg next{};next.circle=true;next.circleLayer=span.layer;next.start=output_.commandMCS;next.end=end;
    next.center=center;next.circleStartAngle=from*angle;next.circleEndAngle=to*angle;next.length=std::fabs(to-from)*span.length;
    const double budget=0.5*(std::min)(request_.accelerationMmS2,request_.decelerationMmS2);
    next.speedLimit=(std::min)((std::min)(feed/60.0,request_.velocityCapMmS),std::sqrt(budget)*std::sqrt(radius));
    next.acceleration=(std::min)(budget,next.speedLimit/acc);next.deceleration=(std::min)(budget,next.speedLimit/dec);
    if(!InsideTravel(end)||!Detail::Positive(next.length)||!Detail::Positive(next.speedLimit)||!StartProfile(next)){Fail(Fault::Numeric);return false;}
    leg_=next;recovery_.evaluationGeometry=&g;feedback_.phase=Phase::RecoveryMove;
    output_.velocityNative.fill(0.0);output_.accelerationNative.fill(0.0);RefreshOutput();return true;
}
inline bool Cycle::BeginRecoveryHold(const Input& input) noexcept
{
    if(feedback_.phase==Phase::GapHistorySettle&&settleAction_==SettleAction::EnterGapHistory){
        feedback_.gapHistoryActive=false;feedback_.phase=feedback_.nominalPhase;
    }
    if(!(Detail::IsNominal(feedback_.phase)||feedback_.phase==Phase::Settle||feedback_.phase==Phase::FlushBrake||
         feedback_.phase==Phase::FlushSettle||Detail::IsFlushMotionPhase(feedback_.phase)||
         feedback_.phase==Phase::GapHistory||feedback_.phase==Phase::GapHistorySettle))return false;
    recovery_.active=true;recovery_.captured=false;recovery_.stage=RecoveryStage::Capture;
    recovery_.sourcePhase=feedback_.phase;recovery_.sourceAction=settleAction_;
    recovery_.sourceFeedback=feedback_;recovery_.inspectionCount=0U;
    feedback_.holdSequence=input.holdSequence;feedback_.holdActive=true;feedback_.inspectionTraceValid=true;
    if(leg_.speed!=0.0){recovery_.brakingCapture=true;return StartBrake(Phase::RecoveryBrake);}
    return CaptureRecoveryStop(input);
}
inline bool Cycle::CaptureRecoveryStop(const Input& input) noexcept
{
    using S=RecoveryStage;
    const auto sourcePhase=recovery_.sourcePhase;
    if(feedback_.flushCheckpointSaved&&request_.retreatMode==2U&&
       (sourcePhase==Phase::FlushPathRetreat||sourcePhase==Phase::FlushPathReturn)){
        feedback_.flushPathStationMm=leg_.position==leg_.length?flushPathLegEnd_:
            flushPathLegStart_+(flushPathLegEnd_-flushPathLegStart_)*(leg_.length==0.0?1.0:leg_.position/leg_.length);
    }
    feedback_.phase=sourcePhase;RefreshOutput();
    recovery_.sourceLeg=leg_;recovery_.sourceFeedback=feedback_;recovery_.sourceFeedback.phase=sourcePhase;
    recovery_.processPoint=output_.commandMCS;recovery_.nominalAnchor=output_.commandMCS;
    recovery_.bridgeGeometry=nullptr;recovery_.bridgeCount=1U;recovery_.bridgePoints[0]=output_.commandMCS;
    Phase nominalPhase=feedback_.nominalPhase;const Leg* nominal=&leg_;
    if(feedback_.gapHistoryActive)nominal=&historyNominalLeg_;
    else if(feedback_.flushCheckpointSaved)nominal=&savedLeg_;
    if(!RecoveryStation(nominalPhase,feedback_.layerIndex,*nominal,recovery_.nominalStation)){Fail(Fault::Numeric);return false;}
    NCG76SphereRetreat::Span nominalSpan{};Point8 center{};
    if(request_.geometry.kind==NCG76SphereRuntimeGeometry::Kind::Line&&request_.geometry.travelMm==0.0)
        recovery_.nominalAnchor=request_.geometry.startMCS;
    else{
        const auto stage=RecoveryNominalStage(nominalPhase);
        if(!NCG76SphereRetreat::GetSpan(request_.geometry,stage,feedback_.layerIndex,nominalSpan)||
           !NCG76SphereRetreat::Evaluate(request_.geometry,nominalSpan,nominal->position/nominal->length,recovery_.nominalAnchor,center))
        {Fail(Fault::Numeric);return false;}
    }
    if(feedback_.flushCheckpointSaved)recovery_.nominalAnchor=savedAnchor_;
    else if(!feedback_.gapHistoryActive)recovery_.nominalAnchor=recovery_.processPoint;
    if(feedback_.gapHistoryActive){
        recovery_.bridgeGeometry=previousPrimitiveLegActive_?&previousPrimitive_:&request_.geometry;
        if(previousPrimitiveLegActive_){
            recovery_.bridgeStartStation=leg_.position;recovery_.bridgeEndStation=previousPrimitive_.travelMm;
        }else{
            const auto stage=RecoveryNominalStage(previousPhase_);
            if(!NCG76SphereRetreat::Station(request_.geometry,stage,previousLayer_,leg_.position/leg_.length,recovery_.bridgeStartStation))
            {Fail(Fault::Numeric);return false;}
            recovery_.bridgeEndStation=recovery_.nominalStation;
        }
    }else if(feedback_.flushActive&&feedback_.flushCheckpointSaved&&request_.retreatMode==2U){
        recovery_.bridgeGeometry=&request_.geometry;recovery_.bridgeStartStation=feedback_.flushPathStationMm;
        recovery_.bridgeEndStation=feedback_.flushAnchorStationMm;
    }else if(feedback_.flushActive&&feedback_.flushCheckpointSaved){
        const bool side=request_.retreatMode>=3U&&
            (sourcePhase==Phase::FlushLift||sourcePhase==Phase::FlushReturn||
             (sourcePhase==Phase::FlushSettle&&(flushRouteIndex_==1U)));
        if(side&&output_.commandMCS!=flushWaypoint_)recovery_.bridgePoints[recovery_.bridgeCount++]=flushWaypoint_;
        if(recovery_.bridgePoints[recovery_.bridgeCount-1U]!=recovery_.nominalAnchor)
            recovery_.bridgePoints[recovery_.bridgeCount++]=recovery_.nominalAnchor;
    }
    recovery_.captured=true;recovery_.brakingCapture=false;
    recovery_.ordinal=recovery_.historyCount;recovery_.pathStation=recovery_.nominalStation;
    recovery_.retreatAnchor=RecoveryGeometry(recovery_.targetOrdinal).startMCS;
    recovery_.stage=S::Capture;StartSettle(Phase::RecoverySettle,SettleAction::RecoveryContinue,input);return !output_.faulted;
}
inline void Cycle::DriveRecoveryPath(const Input& input,bool returning) noexcept
{
    using S=RecoveryStage;
    // Zero-length parents remain counted, but require no invented physical leg.
    for(unsigned hops=0U;hops<17U;++hops){
        const auto& g=RecoveryGeometry(recovery_.ordinal);double total=0.0;
        if(!RecoveryTotal(g,total)){Fail(Fault::Numeric);return;}
        const double target=returning&&recovery_.ordinal==recovery_.historyCount?recovery_.nominalStation:returning?total:0.0;
        if(recovery_.pathStation!=target){
            (void)StartRecoveryPath(g,recovery_.pathStation,target,returning);return;
        }
        if(!returning){
            if(recovery_.ordinal==recovery_.targetOrdinal){
                if(output_.commandMCS!=recovery_.retreatAnchor){Fail(Fault::Numeric);return;}
                recovery_.evaluationGeometry=nullptr;recovery_.stage=S::Held;
                feedback_.phase=Phase::HeldAtStart;heldStartFloor_=input.startSequence;RefreshOutput();return;
            }
            --recovery_.ordinal;
            if(!RecoveryTotal(RecoveryGeometry(recovery_.ordinal),recovery_.pathStation)){Fail(Fault::Numeric);return;}
        }else{
            if(recovery_.ordinal==recovery_.historyCount){
                if(recovery_.bridgeGeometry&&recovery_.bridgeStartStation!=recovery_.bridgeEndStation){
                    recovery_.stage=S::BridgeInPath;recovery_.pathStation=recovery_.bridgeEndStation;
                    (void)StartRecoveryPath(*recovery_.bridgeGeometry,recovery_.pathStation,recovery_.bridgeStartStation,true);return;
                }
                if(recovery_.bridgeCount>1U){
                    recovery_.stage=S::BridgeInLines;recovery_.bridgeIndex=recovery_.bridgeCount-1U;
                    (void)StartRecoveryLine(recovery_.bridgePoints[recovery_.bridgeIndex-1U],true);return;
                }
                RestoreRecoveryCheckpoint(input);return;
            }
            ++recovery_.ordinal;recovery_.pathStation=0.0;
        }
    }
    Fail(Fault::Numeric);
}
inline void Cycle::RestoreRecoveryCheckpoint(const Input& input) noexcept
{
    if(output_.commandMCS!=recovery_.processPoint){Fail(Fault::Numeric);return;}
    const auto hold=feedback_.holdSequence,resume=feedback_.resumeSequence,tick=feedback_.tick;
    feedback_=recovery_.sourceFeedback;feedback_.holdSequence=hold;feedback_.resumeSequence=resume;feedback_.tick=tick;
    leg_=recovery_.sourceLeg;leg_.speed=leg_.accelerationNow=0.0;
    recovery_.active=false;recovery_.stage=RecoveryStage::None;recovery_.evaluationGeometry=nullptr;
    feedback_.holdActive=false;feedback_.held=false;feedback_.gapApplied=false;feedback_.inspectionJogActive=false;
    if(Detail::IsSettling(recovery_.sourcePhase)){
        StartSettle(recovery_.sourcePhase,recovery_.sourceAction,input);return;
    }
    if(recovery_.sourcePhase==Phase::FlushBrake){
        StartSettle(Phase::FlushSettle,SettleAction::CaptureFlush,input);return;
    }
    if(!StartProfile(leg_)){Fail(Fault::Numeric);return;}
    feedback_.phase=recovery_.sourcePhase;
    if(Detail::IsFlushMotionPhase(feedback_.phase)&&!ConfigureFlushProfile(feedback_.phase))return;
    RefreshOutput();
}
inline void Cycle::ContinueRecovery(const Input& input) noexcept
{
    using S=RecoveryStage;
    switch(recovery_.stage){
    case S::Capture:
        if(recovery_.bridgeGeometry&&recovery_.bridgeStartStation!=recovery_.bridgeEndStation){
            recovery_.stage=S::BridgeOutPath;recovery_.pathStation=recovery_.bridgeStartStation;
            (void)StartRecoveryPath(*recovery_.bridgeGeometry,recovery_.pathStation,recovery_.bridgeEndStation,false);return;
        }
        if(recovery_.bridgeCount>1U){
            recovery_.stage=S::BridgeOutLines;recovery_.bridgeIndex=1U;
            (void)StartRecoveryLine(recovery_.bridgePoints[1U],false);return;
        }
        recovery_.stage=S::Retreat;recovery_.ordinal=recovery_.historyCount;recovery_.pathStation=recovery_.nominalStation;
        DriveRecoveryPath(input,false);return;
    case S::BridgeOutPath:
        recovery_.pathStation=recovery_.legEndStation;
        if(recovery_.pathStation!=recovery_.bridgeEndStation){
            (void)StartRecoveryPath(*recovery_.bridgeGeometry,recovery_.pathStation,recovery_.bridgeEndStation,false);return;
        }
        recovery_.stage=S::Retreat;recovery_.ordinal=recovery_.historyCount;recovery_.pathStation=recovery_.nominalStation;
        DriveRecoveryPath(input,false);return;
    case S::BridgeOutLines:
        if(++recovery_.bridgeIndex<recovery_.bridgeCount){(void)StartRecoveryLine(recovery_.bridgePoints[recovery_.bridgeIndex],false);return;}
        recovery_.stage=S::Retreat;recovery_.ordinal=recovery_.historyCount;recovery_.pathStation=recovery_.nominalStation;
        DriveRecoveryPath(input,false);return;
    case S::Retreat:case S::Forward:
        recovery_.pathStation=recovery_.legEndStation;DriveRecoveryPath(input,recovery_.stage==S::Forward);return;
    case S::BridgeInPath:
        recovery_.pathStation=recovery_.legEndStation;
        if(recovery_.pathStation!=recovery_.bridgeStartStation){
            (void)StartRecoveryPath(*recovery_.bridgeGeometry,recovery_.pathStation,recovery_.bridgeStartStation,true);return;
        }
        RestoreRecoveryCheckpoint(input);return;
    case S::BridgeInLines:
        if(--recovery_.bridgeIndex>0U){(void)StartRecoveryLine(recovery_.bridgePoints[recovery_.bridgeIndex-1U],true);return;}
        RestoreRecoveryCheckpoint(input);return;
    case S::JogSettle:
        recovery_.stage=S::Held;recovery_.evaluationGeometry=nullptr;feedback_.phase=Phase::HeldAtStart;
        heldStartFloor_=input.startSequence;RefreshOutput();return;
    case S::JogReturn:
        if(--recovery_.inspectionIndex>0U){
            const auto& next=recovery_.inspectionIndex==1U?recovery_.retreatAnchor:recovery_.inspectionPoints[recovery_.inspectionIndex-2U];
            (void)StartRecoveryLine(next,true,Phase::InspectionJogReturn);return;
        }
        recovery_.inspectionCount=0U;recovery_.stage=S::Forward;recovery_.ordinal=recovery_.targetOrdinal;recovery_.pathStation=0.0;
        DriveRecoveryPath(input,true);return;
    case S::PauseSettle:
        recovery_.stage=S::Paused;feedback_.phase=Phase::HeldAtStart;heldStartFloor_=input.startSequence;RefreshOutput();return;
    default:Fail(Fault::Numeric);return;
    }
}
inline bool Cycle::CanInspectionJog() const noexcept
{
    return recovery_.enabled&&recovery_.active&&recovery_.stage==RecoveryStage::Held&&InspectionTravelAvailable()&&
        feedback_.held&&feedback_.phase==Phase::HeldAtStart&&output_.stopped&&output_.zeroDemand&&
        feedback_.inspectionTraceValid&&recovery_.inspectionCount<recovery_.inspectionPoints.size();
}
inline bool Cycle::BeginInspectionJog(unsigned axis,double signedVelocityMmMin) noexcept
{
    if(!CanInspectionJog()||axis>=3U||!std::isfinite(signedVelocityMmMin)||signedVelocityMmMin==0.0)return false;
    const double end=signedVelocityMmMin>0.0?recovery_.inspectionMaximum[axis]:recovery_.inspectionMinimum[axis];
    if(!std::isfinite(end))return false;
    const double distance=std::fabs(end-output_.commandMCS[axis]);if(!Detail::Positive(distance))return false;
    Leg next{};next.start=next.end=output_.commandMCS;next.end[axis]=end;next.length=distance;
    next.direction[axis]=signedVelocityMmMin>0.0?1.0:-1.0;
    next.speedLimit=(std::min)(std::fabs(signedVelocityMmMin)/60.0,request_.velocityCapMmS);
    next.acceleration=(std::min)(request_.accelerationMmS2,next.speedLimit/recovery_.config.jogAccelSeconds);
    next.deceleration=(std::min)(request_.decelerationMmS2,next.speedLimit/recovery_.config.jogDecelSeconds);
    if(!Detail::Positive(next.speedLimit)||!Detail::Positive(next.acceleration)||!Detail::Positive(next.deceleration))return false;
    recovery_.jogStart=output_.commandMCS;recovery_.inspectionFeed=next.speedLimit*60.0;
    recovery_.jogStop=false;recovery_.evaluationGeometry=nullptr;recovery_.stage=RecoveryStage::Jog;
    leg_=next;feedback_.phase=Phase::InspectionJog;RefreshOutput();return true;
}
inline bool Cycle::EndInspectionJog() noexcept
{
    if(!recovery_.enabled||!recovery_.active)return false;
    if(recovery_.stage==RecoveryStage::JogStopping||recovery_.stage==RecoveryStage::JogSettle)return true;
    if(recovery_.stage!=RecoveryStage::Jog)return false;
    recovery_.jogStop=true;recovery_.stage=RecoveryStage::JogStopping;feedback_.phase=Phase::InspectionJogBrake;RefreshOutput();return true;
}
inline void Cycle::StepRecovery(const Input& input,double dt,bool holdEdge) noexcept
{
    using S=RecoveryStage;
    // A latched inspection interlock may keep the HOLD level high after the
    // initial edge. Consume every START seen behind that fence; releasing the
    // interlock cannot promote an already queued button into a fresh START.
    if(input.hold)heldStartFloor_=(std::max)(heldStartFloor_,input.startSequence);
    if(holdEdge){
        feedback_.holdSequence=input.holdSequence;heldStartFloor_=input.startSequence;
        if(recovery_.stage==S::Jog||recovery_.stage==S::JogStopping)(void)EndInspectionJog();
        else if(feedback_.phase==Phase::RecoveryMove||feedback_.phase==Phase::InspectionJogReturn){
            recovery_.pausedStage=recovery_.stage;recovery_.pausedMovePhase=feedback_.phase;
            recovery_.stage=S::PauseBraking;
            if(leg_.speed!=0.0){(void)StartBrake(Phase::RecoveryBrake);return;}
            recovery_.pausedMoveLeg=leg_;recovery_.stage=S::PauseSettle;
            StartSettle(Phase::RecoverySettle,SettleAction::RecoveryContinue,input);return;
        }
        else if(feedback_.phase==Phase::RecoverySettle&&recovery_.stage!=S::Capture&&
                recovery_.stage!=S::JogSettle&&recovery_.stage!=S::PauseSettle){
            recovery_.pausedStage=recovery_.stage;
            recovery_.pausedMovePhase=recovery_.stage==S::JogReturn?Phase::InspectionJogReturn:Phase::RecoveryMove;
            recovery_.pausedMoveLeg=leg_;recovery_.stage=S::PauseSettle;
            StartSettle(Phase::RecoverySettle,SettleAction::RecoveryContinue,input);return;
        }
    }
    if(feedback_.phase==Phase::RecoveryBrake){
        bool finished=false;if(!AdvanceLeg(dt,true,finished)){Fail(Fault::Numeric);return;}
        if(finished){
            if(recovery_.stage==S::Capture){(void)CaptureRecoveryStop(input);return;}
            if(recovery_.stage==S::PauseBraking){
                recovery_.pausedMoveLeg=leg_;recovery_.stage=S::PauseSettle;
                StartSettle(Phase::RecoverySettle,SettleAction::RecoveryContinue,input);return;
            }
            Fail(Fault::Numeric);return;
        }
        RefreshOutput();return;
    }
    if(feedback_.phase==Phase::RecoverySettle){if(Settled(input,dt))ContinueRecovery(input);RefreshOutput();return;}
    if(recovery_.stage==S::Jog||recovery_.stage==S::JogStopping){
        // An early START is consumed; it can never queue an inspection return.
        heldStartFloor_=(std::max)(heldStartFloor_,input.startSequence);
        bool finished=false;
        if(!AdvanceGap(dt,recovery_.stage==S::Jog?recovery_.inspectionFeed:0.0,finished)){Fail(Fault::Numeric);return;}
        if((recovery_.stage==S::JogStopping&&leg_.speed==0.0)||finished){
            if(output_.commandMCS!=recovery_.jogStart){
                if(recovery_.inspectionCount>=recovery_.inspectionPoints.size()){feedback_.inspectionTraceValid=false;Fail(Fault::Numeric);return;}
                recovery_.inspectionPoints[recovery_.inspectionCount++]=output_.commandMCS;
            }
            recovery_.stage=S::JogSettle;StartSettle(Phase::RecoverySettle,SettleAction::RecoveryContinue,input);return;
        }
        RefreshOutput();return;
    }
    if(feedback_.phase==Phase::HeldAtStart){
        if(!AtCommand(input.actualMCS)){
            if(feedback_.heldRequalificationSequence==(std::numeric_limits<std::uint64_t>::max)()){Fail(Fault::Numeric);return;}
            ++feedback_.heldRequalificationSequence;
            recovery_.stage=recovery_.stage==S::Paused?S::PauseSettle:S::JogSettle;
            StartSettle(Phase::RecoverySettle,SettleAction::RecoveryContinue,input);return;
        }
        if(!input.hold&&input.startSequence>heldStartFloor_){
            feedback_.resumeSequence=input.startSequence;
            if(recovery_.stage==S::Paused){
                recovery_.stage=recovery_.pausedStage;leg_=recovery_.pausedMoveLeg;
                if(!StartProfile(leg_)){Fail(Fault::Numeric);return;}
                feedback_.phase=recovery_.pausedMovePhase;RefreshOutput();return;
            }
            if(recovery_.stage!=S::Held){Fail(Fault::Numeric);return;}
            if(recovery_.inspectionCount){
                recovery_.stage=S::JogReturn;recovery_.inspectionIndex=recovery_.inspectionCount;
                const auto& end=recovery_.inspectionIndex==1U?recovery_.retreatAnchor:recovery_.inspectionPoints[recovery_.inspectionIndex-2U];
                (void)StartRecoveryLine(end,true,Phase::InspectionJogReturn);return;
            }
            recovery_.stage=S::Forward;recovery_.ordinal=recovery_.targetOrdinal;recovery_.pathStation=0.0;
            DriveRecoveryPath(input,true);return;
        }
        RefreshOutput();return;
    }
    if(feedback_.phase==Phase::RecoveryMove||feedback_.phase==Phase::InspectionJogReturn){
        bool finished=false;if(!AdvanceLeg(dt,false,finished)){Fail(Fault::Numeric);return;}
        if(finished)StartSettle(Phase::RecoverySettle,SettleAction::RecoveryContinue,input);
        RefreshOutput();return;
    }
    Fail(Fault::Numeric);
}
} // namespace NCG76SphereCycle
