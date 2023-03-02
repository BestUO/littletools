//
// Created by kongdb on 5/29/18.
//

#include "TextCompare.h"

#include <map>
#include <algorithm>

#include <iostream>
#include <AppConfig.h>
#include <numeric>
#include <valarray>

/*
 * We use 'edit distance' and 'cos similarity' algorithms.
 * And use some smooth algorithms to calculate average score
 * */

template <typename T>
using Table = std::vector<std::vector<T>>;

TextCompare::TextCompare()
    : advance_when_hit_(0.0f),
      use_keyword_match_(false),
      slice_size_(AppConfig::GetInstance()->GetSliceSize())
{}

void TextCompare::SetStopWords(const std::string &stop_words) {
    stop_words_ = stop_words;
}

void TextCompare::SetSymbols(const std::string &symbols) {
    symbols_ = symbols;
}

void TextCompare::SetAdvanceWhenHit(float value) {
    advance_when_hit_ = value;
}

void TextCompare::SetUseKeywordMatch(bool value) {
    use_keyword_match_ = value;
}

//simply spilt char one by one
//TODO(kongdb):Maybe use TF-IDF and fenci algorithm
//If input is "我a爱b南1京", then the return result
//should be {"我","a","爱","b","南","1","京"}
StringArray TextCompare::SplitWord(const std::string &input) {
    StringArray ret;
    for(size_t i = 0; i < input.size();) {
        size_t len = 1;
        //current char maybe chinese character, and size is 3
        if(input[i] < 0) {
            len = kSizePerWord;
        }
        if(i < input.size() - len + 1)
            ret.push_back(input.substr(i, len));
        i += len;
    }
    return ret;
}

//this algorithm is dp
float TextCompare::EditDistance(const StringArray &left_chars,
                                const StringArray &right_chars) {
    size_t left_len = left_chars.size(), right_len = right_chars.size();
    Table<int> dest(left_len + 1, std::vector<int>(right_len + 1));
    for(int i = 0; i <= left_len; ++i)
        dest[i][0] = i;
    for(int i = 0; i <= right_len; ++i)
        dest[0][i] = i;
    for(int i = 1; i <= left_len; ++i) {
        for(int j = 1; j <= right_len; ++j) {
            dest[i][j] = std::min(
              std::min(dest[i - 1][j] + 1, dest[i][j - 1] + 1),
              dest[i - 1][j - 1] +
                (left_chars[i - 1] == right_chars[j - 1] ? 0 : 1)
            );
        }
    }

    int dis = dest[left_len][right_len];
    size_t max_len = std::max(left_len, right_len);
    float distance = (max_len - dis) * 1.0f / max_len;
    distance = (Sigmoid(distance * 4) - 0.5f) * 2;
    return distance;
}

float TextCompare::Sigmoid(float input) {
    return 1.0f / (1.0f + std::exp(-input));
}

/*
   this algorithm use vectors to calculate similarity
   suppose strA is 'abac' and strB is 'acd' then the vectors should be
   chars    :   'a'   'b'   'c'   'd'
   vectorA  : [  2  ,  1  ,  1  ,  0  ]
   vectorB  : [  1  ,  0  ,  1  ,  1  ]
   the similarith between vectorA and vectorB should be:
       vectorA * vectorB       2 * 1 + 1 * 0 + 1 * 1 + 0 * 1
      -------------------   =  -----------------------------   =  0.577
      |vectorA| |vectorB|       sqrt(4+1+1+0)*sqrt(1+0+1+1)
   
   notice:this algorithm ignores the order of words
*/
float TextCompare::CosSimilarity(const StringArray &left_chars,
                                 const StringArray &right_chars) {
    //key is the word string
    //value is the occurance of word in left_chars and right_chars
    std::map<std::string, std::pair<size_t, size_t>> container;

    for(auto &ch : left_chars) {
        ++container[ch].first;
    }

    for(auto &ch : right_chars) {
        ++container[ch].second;
    }

    unsigned long product = 0;
    unsigned long modulo1 = 0;
    unsigned long modulo2 = 0;

    for(auto &entry : container) {
        auto& cnt = entry.second;
        product += cnt.first * cnt.second;
        modulo1 += cnt.first * cnt.first;
        modulo2 += cnt.second * cnt.second;
    }

    auto score = product / (std::sqrt(static_cast<float>(modulo1))
                            * std::sqrt(static_cast<float>(modulo2)));
    return score;
}

float TextCompare::Smooth(float cos, float distance) {
    float target;
    if(distance >= 0.99f) {
        target = 1.0f;
    } else if(distance > 0.9f) {
        target = SimilaritySmooth(cos, 0.05f, distance, 0.05);
    } else if(distance > 0.8f) {
        target = SimilaritySmooth(cos, 0.1f, distance, 0.2f);
    } else if(distance > 0.4f) {
        target = SimilaritySmooth(cos, 0.2f, distance, 0.15f);
    } else if(distance > 0.2f) {
        target = SimilaritySmooth(cos, 0.3f, distance, 0.1f);
    } else {
        target = SimilaritySmooth(cos, 0.4, distance, 0.0);
    }
    return target;
}

float TextCompare::SimilaritySmooth(float cos, float factor1,
                                    float distance, float factor2) {
    float target = cos * factor1 + distance - factor2;
    if(target < 0) {
        target = 0;
    } else if(target > 1) {
        target = 1.0f;
    }
    return target;
}

/* return the count as 'sub' in the 'src'
 * for example, if src is 'abcdabc', and sub is 'abc', should return 2
*/
size_t TextCompare::Count(const std::string &src, const std::string &sub) {
    if(src.empty() || sub.empty())
        return 0;
    size_t count = 0, pos = 0;
    while(true) {
        pos = src.find(sub, pos);
        if(pos != std::string::npos)
            count++;
        else
            break;
        //if src is 'ababa' and sub is 'aba', should return 1 not 2
        pos += sub.size();
    }
    return count;
}

/* clean the meaningless stop words and symbols
 * for example, if input is {'啊', '是', '的', '呢', '。'}
 * the output should be {'是', '的'}
*/
void TextCompare::CleanSentence(StringArray &chars) {
    //only when size greater than '1'
    for(auto itr = chars.begin(); itr != chars.end() && chars.size() > 1;) {
        if((symbols_.find(*itr) != std::string::npos) ||
            (stop_words_.find(*itr) != std::string::npos)) {
            itr = chars.erase(itr);
        } else
            ++itr;
    }
}

float TextCompare::GetCompareResult(const std::string &left,
                                    const std::string &right,const MatchStrategy &strategy) {
    if (left == right)
        return 1.0f;
    else if (left.empty() || right.empty())
        return 0.0f;

    if (use_keyword_match_) {
        if (left.find(right) != std::string::npos) {
            //LOG(INFO) << "'" << left << "' contains '" << right << "'";
            return 1.0;
        }
        return 0.0;
    }

    auto left_chars = SplitWord(left), right_chars = SplitWord(right);
    CleanSentence(left_chars);
    CleanSentence(right_chars);
    if (left_chars == right_chars)
        return 1.0f;

    if (strategy == MatchStrategy::SLICE && slice_size_ > 0 && left_chars.size() > slice_size_) {
        //进行分词匹配
        float final_score = 0.0f;

        //auto to_print = std::accumulate(left_chars.begin(),
        //                                left_chars.end(), std::string(""));
        //LOG(INFO) << "进行分词匹配, 原语句: " << to_print;

        for (auto vec_ite = left_chars.begin(); vec_ite + slice_size_ - 1 < left_chars.end(); ++vec_ite) {

            StringArray slice_left_chars(vec_ite, vec_ite + slice_size_);

            float cos = CosSimilarity(slice_left_chars, right_chars),
                    distance = EditDistance(slice_left_chars, right_chars);

            float my_score = Smooth(cos, distance);
            if (advance_when_hit_ > 0.0f) {
                auto left_cleaned = std::accumulate(slice_left_chars.begin(),
                                                    slice_left_chars.end(), std::string("")),
                        right_cleaned = std::accumulate(right_chars.begin(),
                                                        right_chars.end(), std::string(""));
                size_t hit_count = Count(left_cleaned, right_cleaned);
                my_score += advance_when_hit_ * hit_count;
            }
            /*
            if (my_score > final_score) {
                LOG(INFO) << "slice_left :" << std::accumulate(slice_left_chars.begin(),
                                                               slice_left_chars.end(), std::string("")) << " right : "
                          << std::accumulate(right_chars.begin(),
                                             right_chars.end(), std::string("")) << " score: " << my_score;
            }
             */
            final_score = (my_score > final_score) ? my_score : final_score;
        }
        //LOG(INFO) << "left :" << left << " right : " << right << " final_score: " << final_score;
        //LOG(INFO) << "----------";
        return final_score > 1.0f ? 1.0f : final_score;

    } else {

        float cos = CosSimilarity(left_chars, right_chars),
                distance = EditDistance(left_chars, right_chars);
        //LOG(INFO)<< "left :" << left << " right : " << right
        //         << " cos :" << cos << " distance : " << distance;

        float final_score = Smooth(cos, distance);
        if (advance_when_hit_ > 0.0f) {
            auto left_cleaned = std::accumulate(left_chars.begin(),
                                                left_chars.end(), std::string("")),
                    right_cleaned = std::accumulate(right_chars.begin(),
                                                    right_chars.end(), std::string(""));
            size_t hit_count = Count(left_cleaned, right_cleaned);
            final_score += advance_when_hit_ * hit_count;
        }

        //LOG(INFO)<< "advance the score " << advance_when_hit_*hit_count;

        return final_score > 1.0f ? 1.0f : final_score;
    }
}
