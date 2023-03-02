#ifndef DIALOGMANAGER_TEXTSIMILARITY_H
#define DIALOGMANAGER_TEXTSIMILARITY_H

#include <mutex>
#include <utility>
#include <set>
#include <memory>

#include "TextCompare.h"
#include "AppConfig.h"


class TextSimilarity {
public:
    explicit TextSimilarity(bool use_multithread = false);

    // @param - text_1 is the text to be compared
    //        - text_lst is the list of benchmark texts
    // @param - strategy is the match strategy
    // @return the pair of highest score, and matched string
    //         the score is between 0 and 1. and 1 is best match
    MatchResult GetScore(const std::string &text_1,
                         const std::set<std::string> &text_lst, const MatchStrategy &strategy);

    float UpperBoundScore() const;

    float LowerBoundScore() const;

private:
    MatchResult result_;
    static std::mutex request_mtx_;

    std::shared_ptr<TextCompare> alternative_impl_;
    bool use_multithread_;

    AppConfig *config_;
    float perfect_score_;
    float upper_bound_score_;
    float lower_bound_score_;

    float CompareTwo(const std::string &text_1, const std::string &text_2,
                     const MatchStrategy &strategy);

    typedef std::set<std::string>::const_iterator Iterator;

    void CompareOneByOne(const std::string &text_1,
                         const std::set<std::string> &text_lst, const MatchStrategy &strategy);

    void CompareByMultiThread(const std::string &text_1,
                              const std::set<std::string> &text_lst, const MatchStrategy &strategy);

    //This is a thread function
    void PostRequestInBlock(const std::string &text_1,
                            Iterator first, Iterator last, const MatchStrategy &strategy);
};


#endif //DIALOGMANAGER_TEXTSIMILARITY_H
