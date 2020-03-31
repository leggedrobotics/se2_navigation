/*
 * PathPreprocessor.cpp
 *
 *  Created on: Mar 24, 2020
 *      Author: jelavice
 */

#include "pure_pursuit_core/path_tracking/PathPreprocessor.hpp"

#include <iostream>

#include "pure_pursuit_core/Path.hpp"

namespace pure_pursuit {

bool PathPreprocessor::preprocessPath(Path* path) {
  const int numSegmentsRemoved = removeShortPathSegments(path);
  if (path->segment_.empty()) {
    std::cout << "Path size is 0. All the segments have been erased." << std::endl;
    return false;
  }

  std::cout << "Removed " << numSegmentsRemoved
            << " pah segments, number of remaining path "
               "segments: "
            << path->segment_.size() << std::endl;

  const int numPiecesMerged = mergePathSegmentsWithSameDrivingDirections(path);
  std::cout << "Segments merged: " << numPiecesMerged << ", current number of segments: " << path->segment_.size() << std::endl;

  return true;
}
void PathPreprocessor::setParameters(const PathPreprocessorParameters& parameters) {
  parameters_ = parameters;
}

int PathPreprocessor::removeShortPathSegments(Path* path) {
  auto predicate = [this](const PathSegment& element) {
    const auto start = element.point_.front().position_;
    const auto goal = element.point_.back().position_;
    const double distance = (start - goal).norm();
    const bool isTooShort = distance <= parameters_.minimumSegmentLength_;
    const bool isTooFewElements = element.point_.size() < 2;
    return (isTooShort || isTooFewElements);
  };

  auto& segments = path->segment_;
  const int sizeBeforePruning = segments.size();
  segments.erase(std::remove_if(segments.begin(), segments.end(), predicate), segments.end());

  return sizeBeforePruning - segments.size();
}

int PathPreprocessor::mergePathSegmentsWithSameDrivingDirections(Path* path) {
  Path trajcetoriesCopy = *path;
  path->segment_.clear();

  auto appendPathSegment = [=](const PathSegment& pieceToMerge) -> void {
    PathSegment& currPiece = path->segment_.back();
    currPiece.point_.reserve(currPiece.point_.size() + pieceToMerge.point_.size());
    currPiece.point_.insert(currPiece.point_.end(), pieceToMerge.point_.begin(), pieceToMerge.point_.end());
    assert(currPiece.drivingDirection_ == pieceToMerge.drivingDirection_);
  };

  int piecesGluedTogether = 0;
  auto currPieceIterator = trajcetoriesCopy.segment_.begin();
  // push back the first piece
  path->segment_.push_back(*currPieceIterator);
  ++currPieceIterator;
  for (; currPieceIterator != trajcetoriesCopy.segment_.end(); ++currPieceIterator) {
    // iterate over pieces
    // if the piece is the same as the one that is already merged
    // the glue it together with the previous
    // otherwise
    // push back this new piece in the collection
    const bool isSameDirectionAsPreviousPiece = currPieceIterator->drivingDirection_ == path->segment_.back().drivingDirection_;

    if (isSameDirectionAsPreviousPiece) {
      appendPathSegment(*currPieceIterator);
      ++piecesGluedTogether;
    } else
      path->segment_.push_back(*currPieceIterator);
  }
  return piecesGluedTogether;
}

std::unique_ptr<PathPreprocessor> createPathPreprocessor(const PathPreprocessorParameters& parameters) {
  std::unique_ptr<PathPreprocessor> preprocessor = std::make_unique<PathPreprocessor>();
  preprocessor->setParameters(parameters);
  return std::move(preprocessor);
}

} /* namespace pure_pursuit */
