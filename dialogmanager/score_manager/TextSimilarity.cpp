#include "TextSimilarity.h"

#include <thread>
#include <set>

std::mutex TextSimilarity::request_mtx_;

//It is due to baidu qps limit
//Otherwise will cause "Open api qps request limit reached" error
static constexpr unsigned long kMaxThreadNum = 5;

TextSimilarity::TextSimilarity(bool use_multithread)
      : result_({"", 0.0}),

        alternative_impl_(std::make_shared<TextCompare>()),
        use_multithread_(use_multithread),
        config_(AppConfig::GetInstance()),
        perfect_score_(config_->GetPerfectScore()),
        upper_bound_score_(config_->GetUpperBoundScore()),
        lower_bound_score_(config_->GetLowerBoundScore())

{
    if(alternative_impl_)
    {
        alternative_impl_->SetStopWords(config_->GetStopWords());
        alternative_impl_->SetSymbols(config_->GetSymbols());
        alternative_impl_->SetAdvanceWhenHit(
            config_->GetAdvanceScoreWhenHit());
        alternative_impl_->SetUseKeywordMatch(config_->GetUseKeywordMatch());
    }
}

MatchResult TextSimilarity::GetScore(const std::string &text_1,
                                     const std::set<std::string> &text_lst,const MatchStrategy &strategy) {
    if (text_1.empty() || text_lst.empty())
        return {"", 0.0};

    //reset score to zero
    result_ = {"", 0.0};

    if (use_multithread_)
        CompareByMultiThread(text_1, text_lst, strategy);
    else
        CompareOneByOne(text_1, text_lst, strategy);

    return result_;
}

float TextSimilarity::CompareTwo(const std::string &text_1,
                                 const std::string &text_2,const MatchStrategy &strategy) {
    if(text_1.find("key") == 0)
        return text_1 == text_2;
    else
    {                                     
        if (text_1 == text_2)
            return perfect_score_;
        else if (text_1.empty() || text_2.empty())
            return 0.0;

        return alternative_impl_->GetCompareResult(text_1, text_2, strategy);
    }
}

void TextSimilarity::CompareOneByOne(const std::string &text_1,
                                     const std::set<std::string> &text_lst,const MatchStrategy &strategy)
{
    for(const auto &text_2 : text_lst)
    {
        float score = CompareTwo(text_1, text_2,strategy);
        if(score >= upper_bound_score_)
        {
            result_ = {text_2, score};
            break;
        }
        if(score > result_.second)
        {
            result_ = {text_2, score};
        }
    }
}

void TextSimilarity::PostRequestInBlock(const std::string &text_1,
                                        Iterator first, Iterator last,const MatchStrategy &strategy) {
    for (auto itr = first; itr != last; ++itr) {
        float score = CompareTwo(text_1, *itr, strategy);
        std::lock_guard<std::mutex> guard(request_mtx_);
        if (score > result_.second) {
            result_ = {*itr, score};
        }
    }
}

void TextSimilarity::CompareByMultiThread(const std::string &text_1,
                                          const std::set<std::string> &text_lst,const MatchStrategy &strategy) {
    const unsigned long thread_num = text_lst.size() > kMaxThreadNum ?
                                     kMaxThreadNum : text_lst.size();
    const unsigned long block_size = text_lst.size() / thread_num;
    auto first = text_lst.cbegin(), last = text_lst.end();
    std::vector<std::thread> threads(thread_num - 1);
    auto block_start = first;
    for (int i = 0; i < thread_num - 1; i++) {
        auto block_end = block_start;
        std::advance(block_end, block_size);
        threads[i] = std::thread(&TextSimilarity::PostRequestInBlock,
                                 this, text_1, block_start, block_end, strategy);
        block_start = block_end;
    }
    PostRequestInBlock(text_1, block_start, last, strategy);
    for (auto &t : threads)
        t.join();
}

float TextSimilarity::UpperBoundScore() const
{
    return upper_bound_score_;
}

float TextSimilarity::LowerBoundScore() const
{
    return lower_bound_score_;
}
