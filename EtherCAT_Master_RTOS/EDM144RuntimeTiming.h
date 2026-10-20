#pragma once

#include <cstdint>
#include <limits>

// Diagnostic-only, bounded callback sampling. All durations are nanoseconds.
namespace EDM144
{
    static const std::uint32_t Stride = 65u;
    static const std::uint32_t SamplesPerWindow = 64u;
    static const std::uint32_t PhaseCount = 7u;
    static const std::uint32_t MetricCount = 8u;
    static const std::uint32_t BucketCount = 5u;

    enum Phase
    {
        PreWire = 0,       // callback entry -> transaction envelope
        Wire,              // LRW/FRMW plus existing before/after QPC captures
        PostRx,            // after transaction envelope -> before observation
        Observe,           // before observation -> before interpolation
        Interpolation,     // before interpolation -> after interpolation
        Axes,              // after interpolation -> after axes
        Tail,              // after axes -> callback end
        Total
    };

    enum Bucket
    {
        Idle = 0,
        Auto,
        Other,
        OwnerChange,
        InvalidPDO
    };

    struct Sample
    {
        std::uint64_t at[MetricCount];
        std::uint64_t cycle;
        std::uint32_t ownerBegin;
        std::uint32_t ownerEnd;
        bool processValid;
    };

    struct Stats
    {
        std::uint32_t count;
        std::uint64_t sumNs[MetricCount];
        std::uint64_t maxNs[MetricCount];
        std::uint32_t over250;
    };

    struct Snapshot
    {
        std::uint64_t windowId;
        std::uint64_t firstCycle;
        std::uint64_t lastCycle;
        std::uint32_t attempted;
        std::uint32_t accepted;
        std::uint32_t rejected;
        Stats buckets[BucketCount];
    };

    inline const char* BucketName(unsigned bucket) noexcept
    {
        switch (bucket)
        {
        case Idle: return "IDLE";
        case Auto: return "AUTO";
        case Other: return "OTHER";
        case OwnerChange: return "OWNER_CHANGE";
        case InvalidPDO: return "INVALID_PDO";
        default: return "UNKNOWN";
        }
    }

    class Collector
    {
    public:
        Collector() noexcept : _countdown(0u), _nextWindowId(1u), _window{} {}

        // The first callback is sampled; 65 is coprime with four PLC slots.
        bool Due() noexcept
        {
            if (_countdown == 0u)
            {
                _countdown = Stride - 1u;
                return true;
            }
            --_countdown;
            return false;
        }

        // Output changes only when a complete window of attempted samples ends.
        bool Record(const Sample& sample, Snapshot& output) noexcept
        {
            if (_window.attempted == 0u)
            {
                _window.windowId = _nextWindowId;
                _window.firstCycle = sample.cycle;
            }
            _window.lastCycle = sample.cycle;
            ++_window.attempted;

            bool valid = sample.at[0] != 0u && sample.cycle != 0u;
            for (std::uint32_t phase = 0u; phase < PhaseCount; ++phase)
            {
                if (sample.at[phase + 1u] < sample.at[phase])
                    valid = false;
            }

            // Monotonic timestamps make subtraction safe. Bounding each total
            // also bounds every phase and the sum of up to 64 accepted samples.
            std::uint64_t total = 0u;
            if (valid)
            {
                total = sample.at[PhaseCount] - sample.at[0];
                if (total > (std::numeric_limits<std::uint64_t>::max)() / SamplesPerWindow)
                    valid = false;
            }

            if (valid)
            {
                ++_window.accepted;
                const unsigned bucket = !sample.processValid ? InvalidPDO :
                    sample.ownerBegin != sample.ownerEnd ? OwnerChange :
                    sample.ownerBegin == 0u ? Idle :
                    sample.ownerBegin == 1u ? Auto : Other;
                Stats& stats = _window.buckets[bucket];
                ++stats.count;
                for (std::uint32_t metric = 0u; metric < MetricCount; ++metric)
                {
                    const std::uint64_t duration = metric == Total ? total :
                        sample.at[metric + 1u] - sample.at[metric];
                    stats.sumNs[metric] += duration;
                    if (duration > stats.maxNs[metric])
                        stats.maxNs[metric] = duration;
                }
                if (total > 250000u)
                    ++stats.over250;
            }
            else
            {
                ++_window.rejected;
            }

            if (_window.attempted != SamplesPerWindow)
                return false;

            output = _window;
            _window = Snapshot{};
            ++_nextWindowId;
            if (_nextWindowId == 0u)
                _nextWindowId = 1u;
            return true;
        }

    private:
        std::uint32_t _countdown;
        std::uint64_t _nextWindowId;
        Snapshot _window;
    };

    // Supplied by the runtime's existing snapshot publication channel.
    bool ReadRuntimeTiming(Snapshot& output, std::uint64_t& publication) noexcept;
}


// FIX19: sampled, observational decomposition of the existing axis pass.
// All state below belongs to one Motion writer. No member grants authority.
namespace EDM144 { namespace AxesDetail
{
    static const std::uint32_t Stride = 65u;
    static const std::uint32_t SamplesPerWindow = 64u;
    static const std::uint32_t AxisCapacity = 8u;
    static const std::uint32_t BucketCount = 6u;
    static const std::uint32_t WholePhaseCount = 3u;
    static const std::uint32_t WholeMetricCount = 4u;
    static const std::uint32_t AxisPhaseCount = 7u;
    static const std::uint32_t AxisMetricCount = 8u;
    enum Bucket { NoOwner=0, Auto, AutoZeroOverride, Other, Changed, InvalidRuntime };
    enum WholePhase { Setup=0, Loop, Tail, MotionTotal };
    enum AxisPhase { Input=0, MotionPrelude, ServoPreparation,
        ControllerObservation, OutputGuardsWrite, FeedbackDriveState,
        OutputQualification, SelectedTotal };
    struct Observation
    {
        std::uint64_t cycle;
        std::uint32_t owner;
        bool overrideZero;
        bool runtimeValid;
        std::uint32_t slots;
        int axisIndex;
        const void* axis;
    };
    struct Stats
    {
        std::uint64_t wholeSumNs[WholeMetricCount];
        std::uint64_t wholeMaxNs[WholeMetricCount];
        std::uint64_t axisSumNs[AxisMetricCount];
        std::uint64_t axisMaxNs[AxisMetricCount];
        std::uint32_t count, slotCount, servoCount, completeCount;
        std::uint32_t phaseVisits[AxisPhaseCount];
    };
    struct Snapshot
    {
        std::uint64_t windowId, firstCycle, lastCycle;
        std::uint32_t attempted, accepted, rejected, slot;
        int axisIndex;
        std::uint32_t slots;
        Stats buckets[BucketCount];
    };
    static_assert(sizeof(Snapshot)<=2048u, "Axes detail must fit the existing fixed publication channel.");
    typedef std::uint64_t (*Clock)(void*);
    typedef Observation (*Observer)(void*);
    typedef bool (*Publisher)(void*, const Snapshot&);
    inline const char* BucketName(unsigned bucket) noexcept
    {
        switch(bucket)
        {
        case NoOwner:return "NO_OWNER"; case Auto:return "AUTO";
        case AutoZeroOverride:return "AUTO_ZERO_OVERRIDE";
        case Other:return "OTHER"; case Changed:return "CHANGED";
        case InvalidRuntime:return "INVALID_RUNTIME"; default:return "UNKNOWN";
        }
    }
    class PassScope;
    class SlotScope;
    class Recorder
    {
    public:
        explicit Recorder(Publisher publisher=nullptr, void* context=nullptr) noexcept
            : publisher_(publisher), publicationContext_(context) {}
        std::uint32_t SelectedSlot(std::uint32_t slots) const noexcept
        { return slots!=0u && slots<=AxisCapacity ? nextSlot_%slots : 0u; }
        bool Active() const noexcept { return active_; }
        void MarkWhole(WholePhase phase) noexcept
        {
            if(!active_ || phase>=MotionTotal || phase<=wholePhase_) return;
            const std::uint64_t now=ReadClock();
            Add(wholeNs_[wholePhase_], wholeAt_, now);
            wholeAt_=now; wholePhase_=phase;
        }
        void Mark(const void* axis, AxisPhase phase) noexcept
        {
            if(!IsSelected(axis) || !slotBegun_ || slotEnded_ || phase>=SelectedTotal || phase<=axisPhase_) return;
            const std::uint64_t now=ReadClock();
            Add(axisNs_[axisPhase_], axisAt_, now);
            axisAt_=now; axisPhase_=phase; visited_[phase]=true;
        }
        bool BeginSlot(const void* axis) noexcept
        {
            if(!IsSelected(axis) || slotBegun_) return false;
            slotBegun_=true; axisPhase_=Input; visited_[Input]=true;
            slotAt_=axisAt_=ReadClock();
            return true;
        }
        void CompleteSlot(const void* axis) noexcept
        { if(IsSelected(axis) && slotBegun_ && !slotEnded_) slotComplete_=true; }
        void EndSlot(const void* axis) noexcept
        {
            if(!IsSelected(axis) || !slotBegun_ || slotEnded_) return;
            const std::uint64_t now=ReadClock();
            Add(axisNs_[axisPhase_], axisAt_, now);
            Add(axisNs_[SelectedTotal], slotAt_, now);
            slotEnded_=true;
        }
    private:
        friend class PassScope;
        bool Due() noexcept
        {
            if(countdown_==0u) { countdown_=Stride-1u; return true; }
            --countdown_; return false;
        }
        bool IsSelected(const void* axis) const noexcept
        { return active_ && axis!=nullptr && axis==begin_.axis; }
        static std::uint64_t Limit() noexcept
        { return (std::numeric_limits<std::uint64_t>::max)()/SamplesPerWindow; }
        std::uint64_t ReadClock() noexcept
        {
            const std::uint64_t now=clock_(clockContext_);
            if(now==0u || (lastClock_!=0u && now<lastClock_)) bad_=true;
            lastClock_=now; return now;
        }
        void Add(std::uint64_t& value, std::uint64_t before, std::uint64_t after) noexcept
        {
            if(before==0u || after<before || after-before>Limit() || value>Limit()-(after-before))
            { bad_=true; return; }
            value+=after-before;
        }
        bool Begin(Clock clock,void* clockContext,Observer observer,void* observationContext) noexcept
        {
            if(active_ || clock==nullptr || observer==nullptr || !Due()) return false;
            clock_=clock; clockContext_=clockContext; observer_=observer; observationContext_=observationContext;
            begin_=observer_(observationContext_);
            active_=true; bad_=false; lastClock_=0u;
            slotBegun_=slotEnded_=slotComplete_=false;
            wholePhase_=Setup; axisPhase_=Input;
            for(unsigned i=0u;i<WholeMetricCount;++i) wholeNs_[i]=0u;
            for(unsigned i=0u;i<AxisMetricCount;++i) axisNs_[i]=0u;
            for(unsigned i=0u;i<AxisPhaseCount;++i) visited_[i]=false;
            motionAt_=wholeAt_=ReadClock();
            return true;
        }
        void End() noexcept
        {
            if(!active_) return;
            if(slotBegun_ && !slotEnded_) EndSlot(begin_.axis);
            const Observation end=observer_(observationContext_);
            const std::uint64_t now=ReadClock();
            Add(wholeNs_[wholePhase_],wholeAt_,now);
            Add(wholeNs_[MotionTotal],motionAt_,now);
            active_=false;
            const std::uint32_t slot=SelectedSlot(begin_.slots);
            if(window_.attempted==0u)
            {
                window_.windowId=nextWindowId_;
                window_.firstCycle=begin_.cycle;
                window_.slot=slot; window_.axisIndex=begin_.axisIndex; window_.slots=begin_.slots;
                windowAxis_=begin_.axis;
            }
            window_.lastCycle=begin_.cycle; ++window_.attempted;
            bool valid=!bad_ && begin_.cycle!=0u && end.cycle==begin_.cycle &&
                begin_.slots<=AxisCapacity && end.slots==begin_.slots &&
                end.axisIndex==begin_.axisIndex && end.axis==begin_.axis &&
                window_.slot==slot && window_.axisIndex==begin_.axisIndex &&
                window_.slots==begin_.slots && windowAxis_==begin_.axis;
            if((begin_.slots==0u)!=(begin_.axis==nullptr)) valid=false;
            if(begin_.slots!=0u && (begin_.axisIndex<0 || begin_.axisIndex>=static_cast<int>(AxisCapacity))) valid=false;
            std::uint64_t wholeSum=0u, axisSum=0u;
            for(unsigned i=0u;i<WholePhaseCount;++i) wholeSum+=wholeNs_[i];
            for(unsigned i=0u;i<AxisPhaseCount;++i) axisSum+=axisNs_[i];
            valid=valid && wholeSum==wholeNs_[MotionTotal] && axisSum==axisNs_[SelectedTotal] &&
                axisNs_[SelectedTotal]<=wholeNs_[MotionTotal];
            if(valid)
            {
                ++window_.accepted;
                const unsigned bucket=(!begin_.runtimeValid || !end.runtimeValid)?InvalidRuntime:
                    (begin_.owner!=end.owner || begin_.overrideZero!=end.overrideZero)?Changed:
                    begin_.owner==0u?NoOwner:begin_.owner==1u?(begin_.overrideZero?AutoZeroOverride:Auto):Other;
                Stats& stats=window_.buckets[bucket]; ++stats.count;
                if(slotBegun_) ++stats.slotCount;
                if(visited_[ServoPreparation]) ++stats.servoCount;
                if(slotComplete_) ++stats.completeCount;
                for(unsigned i=0u;i<AxisPhaseCount;++i) if(visited_[i]) ++stats.phaseVisits[i];
                for(unsigned i=0u;i<WholeMetricCount;++i)
                { stats.wholeSumNs[i]+=wholeNs_[i]; if(wholeNs_[i]>stats.wholeMaxNs[i]) stats.wholeMaxNs[i]=wholeNs_[i]; }
                for(unsigned i=0u;i<AxisMetricCount;++i)
                { stats.axisSumNs[i]+=axisNs_[i]; if(axisNs_[i]>stats.axisMaxNs[i]) stats.axisMaxNs[i]=axisNs_[i]; }
            }
            else ++window_.rejected;
            if(window_.attempted==SamplesPerWindow)
            {
                completed_=window_;
                if(publisher_!=nullptr) (void)publisher_(publicationContext_,completed_);
                const std::uint32_t slots=begin_.slots<=AxisCapacity?begin_.slots:0u;
                nextSlot_=slots!=0u?(slot+1u)%slots:0u;
                window_=Snapshot{}; windowAxis_=nullptr;
                ++nextWindowId_; if(nextWindowId_==0u) nextWindowId_=1u;
            }
        }
        Publisher publisher_=nullptr; void* publicationContext_=nullptr;
        Clock clock_=nullptr; void* clockContext_=nullptr;
        Observer observer_=nullptr; void* observationContext_=nullptr;
        Observation begin_{};
        Snapshot window_{},completed_{};
        const void* windowAxis_=nullptr;
        std::uint32_t countdown_=0u,nextSlot_=0u;
        std::uint64_t nextWindowId_=1u,lastClock_=0u,motionAt_=0u,wholeAt_=0u,axisAt_=0u,slotAt_=0u;
        std::uint64_t wholeNs_[WholeMetricCount]{},axisNs_[AxisMetricCount]{};
        bool visited_[AxisPhaseCount]{};
        WholePhase wholePhase_=Setup; AxisPhase axisPhase_=Input;
        bool active_=false,bad_=false,slotBegun_=false,slotEnded_=false,slotComplete_=false;
    };
    class PassScope
    {
    public:
        PassScope(Recorder& recorder,Clock clock,void* clockContext,Observer observer,void* observationContext) noexcept
            : recorder_(recorder),owns_(recorder.Begin(clock,clockContext,observer,observationContext)) {}
        ~PassScope() noexcept { if(owns_) recorder_.End(); }
        PassScope(const PassScope&)=delete; PassScope& operator=(const PassScope&)=delete;
    private: Recorder& recorder_; bool owns_;
    };
    class SlotScope
    {
    public:
        SlotScope(Recorder& recorder,const void* axis) noexcept
            : recorder_(recorder),axis_(axis),owns_(recorder.BeginSlot(axis)) {}
        ~SlotScope() noexcept { if(owns_) recorder_.EndSlot(axis_); }
        void Complete() noexcept { if(owns_) recorder_.CompleteSlot(axis_); }
        SlotScope(const SlotScope&)=delete; SlotScope& operator=(const SlotScope&)=delete;
    private: Recorder& recorder_; const void* axis_; bool owns_;
    };
    bool Read(Snapshot& output,std::uint64_t& publication) noexcept;
} }
