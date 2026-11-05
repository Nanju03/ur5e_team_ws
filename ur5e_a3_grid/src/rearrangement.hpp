// SPDX-License-Identifier: BSD-3-Clause
// Geometry-independent tower bookkeeping and bounded rearrangement search.
#pragma once
#include <algorithm>
#include <functional>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace rearrangement {
inline constexpr const char* BUFFER_SELECTION_POLICY="nearest_target_feasible_v1";
struct Move {int block,from,to;bool temporary;};
struct Layout {
  int target=0;
  std::map<int,std::vector<int>> towers;
  std::set<int> parked_once;
  std::pair<int,int> locate(int block)const {
    for(const auto& [cell,tower]:towers)
      for(std::size_t tier=0;tier<tower.size();++tier)
        if(tower[tier]==block)return {cell,static_cast<int>(tier)};
    throw std::runtime_error("Unknown rearrangement block");
  }
  std::string key(bool unordered_target=false)const {
    std::ostringstream out;
    for(const auto& [cell,tower]:towers) {
      out<<cell<<':';
      if(unordered_target&&cell==target) {
        auto contents=tower;std::sort(contents.begin(),contents.end());
        for(int block:contents)out<<block<<',';
      }else for(int block:tower)out<<block<<',';
      out<<';';
    }
    out<<'|';for(int block:parked_once)out<<block<<',';return out.str();
  }
  Layout moved(const Move& move)const {
    Layout next=*this;auto& from=next.towers.at(move.from);auto& to=next.towers.at(move.to);
    if(move.from==target||move.from==move.to||from.empty()||from.back()!=move.block||
       move.temporary!=(move.to!=target)||
       (move.temporary&&(from.size()!=1||to.size()!=1||parked_once.count(move.block))))
      throw std::runtime_error("Invalid rearrangement move or occupied support");
    from.pop_back();to.push_back(move.block);
    if(move.temporary)next.parked_once.insert(move.block);
    return next;
  }
};

class Search {
public:
  using Pick=std::function<bool(const Layout&,int)>;
  using Place=std::function<bool(const Layout&,const Move&)>;
  using BufferDistance=std::function<double(int)>;
  using StateKey=std::function<std::string(const Layout&)>;
  Search(std::vector<int> priority,Pick pick,Place place,BufferDistance buffer_distance,
         StateKey key=[](const Layout& layout){return layout.key();})
    :priority_(std::move(priority)),pick_(std::move(pick)),place_(std::move(place)),
     buffer_distance_(std::move(buffer_distance)),state_key_(std::move(key)){}
  std::vector<Move> plan(const Layout& layout) {
    expanded_=0;
    std::size_t remaining=0;
    for(const auto& [cell,tower]:layout.towers)if(cell!=layout.target)
      for(int block:tower)if(!layout.parked_once.count(block))++remaining;
    // First allow at most one new temporary move. Otherwise a near-target
    // support can spawn a long cascade before the next feasible support is
    // considered. Direct target moves still take priority in every pass.
    for(std::size_t extra=1;extra<=std::max(std::size_t(1),remaining);++extra) {
      route_.clear();failed_.clear();buffer_limit_=layout.parked_once.size()+extra;
      if(visit(layout))return route_;
    }
    throw std::runtime_error(
      "No collision-free rearrangement with the open gripper and recoverable two-layer buffers; NOTHING executed");
  }
  std::size_t expanded()const{return expanded_;}
private:
  bool visit(const Layout& layout) {
    const auto key=state_key_(layout);if(failed_.count(key))return false;
    if(++expanded_>5000)throw std::runtime_error("Rearrangement search limit reached; NOTHING executed");
    std::vector<int> tops;
    for(const auto& [cell,tower]:layout.towers)if(cell!=layout.target&&!tower.empty())tops.push_back(tower.back());
    if(tops.empty())return true;
    auto rank=[this](int id){return std::find(priority_.begin(),priority_.end(),id)-priority_.begin();};
    std::stable_sort(tops.begin(),tops.end(),[&](int a,int b){
      const auto ta=layout.locate(a).second,tb=layout.locate(b).second;
      return ta!=tb?ta>tb:rank(a)<rank(b);
    });
    std::vector<int> accessible;
    for(int id:tops)if(pick_(layout,id))accessible.push_back(id);
    std::vector<int> threatened;
    // Grow the target only while retaining access to every other exposed block.
    // This detects dependencies from actual geometry, with no special cell IDs.
    for(int id:accessible) {
      Move move{id,layout.locate(id).first,layout.target,false};auto next=layout.moved(move);
      bool safe=true;
      for(int other:accessible)if(other!=id&&!pick_(next,other)) {
        safe=false;if(std::find(threatened.begin(),threatened.end(),other)==threatened.end())threatened.push_back(other);
      }
      if(safe&&place_(layout,move)&&descend(next,move))return true;
    }
    if(layout.parked_once.size()>=buffer_limit_){failed_.insert(key);return false;}
    // Park endangered ground blocks first, on an existing single block. A
    // support cannot be picked until its parked top has been retrieved.
    std::vector<int> candidates=threatened;
    for(int id:accessible)if(std::find(candidates.begin(),candidates.end(),id)==candidates.end())candidates.push_back(id);
    std::vector<int> supports;
    for(const auto& [cell,tower]:layout.towers)if(cell!=layout.target&&tower.size()==1)supports.push_back(cell);
    // Prefer the support nearest the final target. Feasibility checks below
    // still reject covered/blocked/unrecoverable candidates before use.
    std::stable_sort(supports.begin(),supports.end(),[&](int a,int b){return buffer_distance_(a)<buffer_distance_(b);});
    for(int id:candidates) {
      if(layout.locate(id).second!=0||layout.parked_once.count(id)||
         std::find(accessible.begin(),accessible.end(),id)==accessible.end())continue;
      for(int support:supports) {
        if(support==layout.locate(id).first)continue;
        Move move{id,layout.locate(id).first,support,true};auto next=layout.moved(move);
        if(!pick_(next,id))continue;
        bool safe=true;
        const int covered=layout.towers.at(support).back();
        for(int other:accessible)if(other!=id&&other!=covered&&!pick_(next,other)){safe=false;break;}
        if(safe&&place_(layout,move)&&descend(next,move))return true;
      }
    }
    failed_.insert(key);return false;
  }
  bool descend(const Layout& next,const Move& move) {
    route_.push_back(move);if(visit(next))return true;route_.pop_back();return false;
  }
  std::vector<int> priority_;Pick pick_;Place place_;BufferDistance buffer_distance_;
  StateKey state_key_;
  std::vector<Move> route_;std::set<std::string> failed_;std::size_t expanded_=0,buffer_limit_=0;
};
} // namespace rearrangement
