#pragma once
#include <cstdint>
#include <limits>

// Observation only. A pointer is supplied only by the PDO owner's selected
// FIX19 pass. Null callers (including NC producers) never access this recorder.
namespace EDM174Leaf
{
    static const std::uint32_t SamplesPerWindow = 64U;
    static const std::uint32_t Stride = 65U;
    static const std::uint32_t MaximumNewClockReads = 6U;
    static const std::uint32_t MaximumCombinedClockReads = 18U;
    enum Site : std::uint32_t { PreludeAuth=0U, Participation, GainAuth, WriterAuth, SiteCount };
    enum Metric : std::uint32_t { Total=0U, Read128, FinalTranslation, Other, MetricCount };
    struct Context
    {
        std::uint64_t cycle=0ULL, session=0ULL;
        std::uint32_t phase=0U, child=0U;
    };
    struct Observation
    {
        Context context{};
        const void* axis=nullptr;
        std::uint32_t slot=0U, slots=0U;
        int axisIndex=-1;
        bool enabled=false, runtimeValid=false;
    };
    struct Stats
    {
        std::uint64_t sumNs[MetricCount]{}, maxNs[MetricCount]{};
        Context maxContext[MetricCount]{};
        std::uint64_t totalMaxSampleNs[MetricCount]{};
        std::uint32_t metricVisits[MetricCount]{};
        std::uint32_t attempted=0U, valid=0U, missing=0U, invalid=0U;
    };
    struct Snapshot
    {
        std::uint64_t windowId=0ULL, firstCycle=0ULL, lastCycle=0ULL;
        Context context{}; // Every accepted sample has this session/phase/child.
        std::uint64_t droppedPublications=0ULL;
        std::uint32_t attempted=0U, enabled=0U, valid=0U, missing=0U, invalid=0U;
        std::uint32_t phaseChanged=0U, contextRejected=0U, phaseMask=0U;
        std::uint32_t clockReads=0U, maxClockReads=0U, slot=0U, slots=0U;
        int axisIndex=-1;
        Stats stats[SiteCount]{};
    };
    static_assert(sizeof(Snapshot)<=4096U, "FIX6 leaf publication is fixed and bounded");
    typedef std::uint64_t (*Clock)(void*);
    typedef Observation (*Observer)(void*);
    typedef bool (*Publisher)(void*,const Snapshot&);
    bool Read(Snapshot& snapshot,std::uint64_t& publication) noexcept;
    inline const char* SiteName(unsigned site) noexcept
    {
        switch(site) { case PreludeAuth:return "PRELUDE_AUTH";case Participation:return "PARTICIPATION";
        case GainAuth:return "GAIN_AUTH";case WriterAuth:return "WRITER_AUTH";default:return "UNKNOWN"; }
    }
    inline bool SameContext(const Context& a,const Context& b) noexcept
    { return a.session==b.session && a.phase==b.phase && a.child==b.child; }

    class Recorder;
    class Scope;
    class Sample
    {
    public:
        Sample* ForSite(Site site) noexcept { return enabled_ && site==site_ ? this : nullptr; }
    private:
        friend class Recorder;
        friend class Scope;
        void Reset(Site site,bool enabled,Clock clock,void* clockContext) noexcept
        {
            site_=site;enabled_=enabled;clock_=clock;clockContext_=clockContext;
            reached_=complete_=bad_=false;clockReads_=0U;lastClock_=0ULL;
            for(unsigned i=0U;i<MetricCount;++i) { start_[i]=value_[i]=0ULL;open_[i]=visited_[i]=false; }
        }
        std::uint64_t Now() noexcept
        {
            if(!clock_ || clockReads_>=MaximumNewClockReads) {bad_=true;return 0ULL;}
            ++clockReads_;const std::uint64_t now=clock_(clockContext_);
            if(!now || (lastClock_ && now<lastClock_))bad_=true;
            lastClock_=now;return now;
        }
        bool Begin(Metric metric) noexcept
        {
            if(!enabled_)return false;
            if(metric>=Other || (site_==Participation && metric!=Total) || open_[metric] || visited_[metric] || complete_ ||
                (metric==Total?reached_:!open_[Total]) ||
                (metric!=Total && (open_[Read128] || open_[FinalTranslation]))) {bad_=true;return false;}
            if(metric==Total)reached_=true;
            open_[metric]=visited_[metric]=true;start_[metric]=Now();return true;
        }
        void End(Metric metric) noexcept
        {
            if(metric>=Other || !open_[metric]) {bad_=true;return;}
            const std::uint64_t end=Now();open_[metric]=false;
            if(!start_[metric] || end<start_[metric])bad_=true;
            else value_[metric]=end-start_[metric];
            if(metric==Total) {
                complete_=true;
                if(open_[Read128] || open_[FinalTranslation] ||
                    value_[Read128]>(std::numeric_limits<std::uint64_t>::max)()-value_[FinalTranslation])bad_=true;
                else {
                    const std::uint64_t nested=value_[Read128]+value_[FinalTranslation];
                    if(nested>value_[Total])bad_=true;
                    else if(site_!=Participation) {value_[Other]=value_[Total]-nested;visited_[Other]=true;}
                }
            }
        }
        Site site_=PreludeAuth;
        bool enabled_=false,reached_=false,complete_=false,bad_=false;
        bool open_[MetricCount]{},visited_[MetricCount]{};
        std::uint32_t clockReads_=0U;
        std::uint64_t lastClock_=0ULL,start_[MetricCount]{},value_[MetricCount]{};
        Clock clock_=nullptr;void* clockContext_=nullptr;
    };
    class Scope
    {
    public:
        Scope(Sample* sample,Metric metric=Total) noexcept : sample_(sample),metric_(metric),begun_(sample && sample->Begin(metric)) {}
        ~Scope() noexcept {if(begun_)sample_->End(metric_);}
        Scope(const Scope&)=delete;Scope& operator=(const Scope&)=delete;
    private:
        Sample* sample_;Metric metric_;bool begun_;
    };
    class PassScope;
    class Recorder
    {
    public:
        explicit Recorder(Publisher publisher=nullptr,void* context=nullptr) noexcept : publisher_(publisher),publicationContext_(context) {}
        Sample* ForAxis(const void* axis) noexcept
        { return active_ && sample_.enabled_ && axis && axis==begin_.axis ? &sample_ : nullptr; }
    private:
        friend class PassScope;
        bool Begin(bool due,Clock clock,void* clockContext,Observer observer,void* observationContext) noexcept
        {
            if(active_ || !due || !observer)return false;
            observer_=observer;observationContext_=observationContext;begin_=observer_(observationContext_);active_=true;
            if(!window_.attempted) {
                window_.windowId=nextWindowId_;window_.firstCycle=begin_.context.cycle;
                window_.slot=begin_.slot;window_.slots=begin_.slots;window_.axisIndex=begin_.axisIndex;
                windowAxis_=begin_.axis;haveContext_=false;
            }
            window_.lastCycle=begin_.context.cycle;
            const Site site=static_cast<Site>(window_.attempted%SiteCount);++window_.attempted;
            eligible_=begin_.enabled;contextRejected_=false;
            if(eligible_) {
                ++window_.enabled;
                if(begin_.context.phase<32U)window_.phaseMask|=1U<<begin_.context.phase;
                if(!haveContext_) {window_.context=begin_.context;haveContext_=true;}
                else if(!SameContext(window_.context,begin_.context)) {++window_.contextRejected;contextRejected_=true;}
            }
            const bool valid=eligible_ && !contextRejected_ && begin_.runtimeValid && begin_.context.cycle && begin_.context.session &&
                begin_.axis && begin_.slots==6U && begin_.slot<6U && begin_.axisIndex>=0 && begin_.axisIndex<6 &&
                window_.slot==begin_.slot && window_.slots==begin_.slots && window_.axisIndex==begin_.axisIndex && windowAxis_==begin_.axis;
            sample_.Reset(site,valid,clock,clockContext);
            return true;
        }
        void End() noexcept
        {
            if(!active_)return;
            const Observation end=observer_(observationContext_);active_=false;
            if(eligible_ && !contextRejected_) {
                Stats& stats=window_.stats[sample_.site_];++stats.attempted;
                window_.clockReads+=sample_.clockReads_;
                if(sample_.clockReads_>window_.maxClockReads)window_.maxClockReads=sample_.clockReads_;
                const bool changed=!SameContext(begin_.context,end.context);
                if(changed)++window_.phaseChanged;
                bool valid=sample_.enabled_ && !sample_.bad_ && !changed && end.enabled && end.runtimeValid &&
                    end.context.cycle==begin_.context.cycle && end.axis==begin_.axis && end.slot==begin_.slot &&
                    end.slots==begin_.slots && end.axisIndex==begin_.axisIndex;
                if(valid && !sample_.reached_) {++stats.missing;++window_.missing;}
                else {
                    valid=valid && sample_.complete_;
                    for(unsigned i=0U;valid && i<MetricCount;++i)
                        if(sample_.visited_[i] && sample_.value_[i]>(std::numeric_limits<std::uint64_t>::max)()-stats.sumNs[i])valid=false;
                    if(!valid) {++stats.invalid;++window_.invalid;}
                    else {
                        ++stats.valid;++window_.valid;
                        const bool totalMax=sample_.value_[Total]>stats.maxNs[Total] || stats.metricVisits[Total]==0U;
                        for(unsigned i=0U;i<MetricCount;++i)if(sample_.visited_[i]) {
                            stats.sumNs[i]+=sample_.value_[i];
                            if(!stats.metricVisits[i] || sample_.value_[i]>stats.maxNs[i]) {stats.maxNs[i]=sample_.value_[i];stats.maxContext[i]=begin_.context;}
                            ++stats.metricVisits[i];
                        }
                        if(totalMax)for(unsigned i=0U;i<MetricCount;++i)stats.totalMaxSampleNs[i]=sample_.value_[i];
                    }
                }
            }
            sample_.enabled_=false;
            if(window_.attempted==SamplesPerWindow) {
                window_.droppedPublications=droppedPublications_;
                if(window_.enabled && (!publisher_ || !publisher_(publicationContext_,window_)))++droppedPublications_;
                window_=Snapshot{};windowAxis_=nullptr;haveContext_=false;
                if(nextWindowId_!=(std::numeric_limits<std::uint64_t>::max)())++nextWindowId_;
            }
        }
        Publisher publisher_=nullptr;void* publicationContext_=nullptr;
        Observer observer_=nullptr;void* observationContext_=nullptr;
        bool active_=false,eligible_=false,contextRejected_=false,haveContext_=false;
        std::uint64_t nextWindowId_=1ULL,droppedPublications_=0ULL;
        const void* windowAxis_=nullptr;
        Observation begin_{};Sample sample_{};Snapshot window_{};
    };
    class PassScope
    {
    public:
        PassScope(Recorder& recorder,bool due,Clock clock,void* clockContext,Observer observer,void* observationContext) noexcept : recorder_(recorder),owns_(recorder_.Begin(due,clock,clockContext,observer,observationContext)) {}
        ~PassScope() noexcept {if(owns_)recorder_.End();}
        PassScope(const PassScope&)=delete;PassScope& operator=(const PassScope&)=delete;
    private:Recorder& recorder_;bool owns_;
    };
}
