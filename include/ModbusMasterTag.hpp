//
// Created by kdluzynski on 06.10.2025.
//

#pragma once

#include <algorithm>
#include <mutex>
#include <set>
#include <variant>
#include "../config.hpp"
#include "ModbusTag.hpp"
#include "ModbusTagValue.hpp"
#include "ModbusMasterBase.hpp"
#include "ModbusRegisterBuffer.hpp"
/** Use Cases
 * 1. Aktualizacja read wartości - może byc asynchronicznie. Gdy przyjda nowe wartosci to tylko aktualizacja w modelu
 * 2. Synchroniczna sekwencja - np. write, read, write np. przeprowadzenie backupu
 *      - to albo synchroniczne akcje - ale to bedzie powodowac problem z thread affinity w Qt
 *      - albo zwracamy promise/future - ale to nie bedzie dzialalo w przypadku embedded bo tam nie ma
 * 3. Wpisywanie czegoś do wielu rejestrów na raz
 * 4. Odczyt z wielu rejestrów na raz
 * 5. Wymiana jakiegoś bufora np. pliku. Czyli wpisujemy w wiele różnych typów na raz wartości.
 *
 * To może w takim razie taki updejt poprzedniego sterownika, że wpisujemy do bufora, ale na podstawie Tagu.
 * Czyli tworzymy taki writer/builder requestów, czyli wpisujemy wartości jakie chcemy wpisać, albo odczytać
 * On buduje na tej podstawie requesty i później możemy je sobie wykonywać.
 * Tylko jak później odczytać z nich wartości?
 * Najlepiej pewnie, jakby otrzymywać w odpowiedzi taki bufor, z którego można by było odczytać wartości pojedynczo
 * odnosząc się do tagów np. value = answer[CID_TAG1] i on by automatycznie konwertował to do naszych wartości
 *
 * A później można by to było łatwo przekształcić w polling?
 * Trzeba by było mieć jakiś cache. Te answery trzeba by było móc jakoś łączyć. Tylko jak?
 */
namespace TagModbus {
	struct Tag;

	class MasterTag : public MasterBase {
	public:
		using MasterBase::MasterBase;
		using TagMap = std::unordered_map<TagID, size_t>;
		using TagValueMap = std::map<TagID, TagValue>;
		using WriteErrors = std::vector<std::pair<TagID, ModbusException>>;
		struct Request {
			RegisterType registerType;
			uint16_t startAddress;
			uint16_t quantity;
			std::vector<TagID> tagIDs;
		};

		using RequestCache = std::unordered_map<size_t, std::vector<Request> >;
		RequestCache _requestCache;

		void registerTags(const std::span<const Tag> tagsToRegister) {
			// OK, wiec tagi w bazie danych musza byc koniecznie posortowane wedlug typu rejestru i numeru
			// Chyba ze zrobic osobny vektor/multimape ktory bedzie tak posortowany i bedzie sie odnosil do mapy z tagid
			// for (const auto &[id, tag] : tagsToRegister) {
			//     tagsDatabase.insert_or_assign(id, tag);
			// }
			clearTags();
			std::ranges::copy(tagsToRegister.begin(),tagsToRegister.end(),_tagsDatabase.end());
			std::ranges::sort(_tagsDatabase, [](const Tag &a, const Tag &b) {
				return (a.register_type < b.register_type) ||
				       (a.register_type == b.register_type && a.register_number < b.register_number);
			});

			for (size_t i = 0; i < _tagsDatabase.size(); ++i) {
				// Use the element's key and its new index 'i'
				_IDtoTagMap.emplace(TagID{_tagsDatabase[i].key}, i);
			}
		}

		void clearTags() {
			_IDtoTagMap.clear();
			_tagsDatabase.clear();
			_requestCache.clear();
			// excludedRegisters.clear();
		}

		void runPolling();

		using TagRef = std::reference_wrapper<const Tag>;

		// RegisterBuffer read(const uint8_t slaveID, std::span<const TagRef> tags) {
		//     std::vector<Request> requests = prepareReadRequests(tags);
		//     std::vector<RegisterBuffer> resp;
		//     for (auto [registerType, startAddress, quantity,tagIDs]: requests) {
		//         RegisterBuffer& buf = resp.emplace_back(startAddress,registerType,quantity);
		//         MasterBase::read(slaveID,buf.view());
		//     }
		//     return resp;
		// }
		//
		// TagValueMap read(const uint8_t slaveID, std::initializer_list<TagRef> tags) {
		//     return read(slaveID, std::span(tags));
		// }

		TagValueMap read(std::initializer_list<const TagID> tags) {
			return read( std::span(tags));
		}
		TagValue read(const TagID tagID) {
			return read({tagID})[tagID];
		}

		TagValueMap read(const std::span<const TagID> tagIDs) {
			TagValueMap result;
			std::vector<Request> requests = prepareReadRequests(tagIDs);
			for (auto &[registerType, startAddress, quantity,tags]: requests) {
				try {
					auto response = MasterBase::read(_slaveID, registerType, startAddress,
					                                 quantity);
					RegisterBufferView parser{startAddress, registerType, response};
					//associate TagID with value
					for (auto &tagID: tags) {
						auto &tag = getTag(tagID);
						result.emplace(tagID, parser.get<TagValue>(tag));
					}
				} catch (ModbusException &e) {
					if (e._exception_code == FrameView::IllegalDataAddress||e._exception_code == FrameView::IllegalFunction) {
						// excludedRegisters[registerType].push_back(startAddress);
					}
				} catch (const std::exception &e) {
					printf("Modbus exception: %s\n", e.what());
				}
			}
			return result;
		}

		// TagValueMap read(std::initializer_list<TagID>tagIDs) {
		//     //check cache for ready requests
		//
		//     //if none is found, then prepare requests
		//     std::vector<RegisterBufferView> requests = prepare_requests(tagsIDs);
		//     //send all the requests
		//     for (const auto& request : requests) {
		//         readRegisters(request.)
		//     }
		//
		//     //return map
		// }
		[[nodiscard]] WriteErrors write(const TagValueMap& values, bool oneByOne = true) {
			WriteErrors writeErrors;
			if (oneByOne) {
				for (auto &[tagID,value]: values) {
					const auto &info = getTag(tagID);
					try {
						MasterBase::write(_slaveID, info.register_type, info.register_number, value.data());
					}catch (const ModbusException& modbus_exception) {
						writeErrors.push_back({tagID, modbus_exception});
					}
				}
			}else {
				//TODO implement optimal write requests
			}
			return writeErrors;
		}

		static MasterTag TCP(IStreamDevice &serial_device) {
			MasterTag result(serial_device);
			result.isTCP = true;
			return result;
		}

		static MasterTag RTU(IStreamDevice &serial_device) {
			MasterTag result(serial_device);
			result.isTCP = false;
			return result;
		}

		[[nodiscard]] uint8_t slaveID() const {
			return _slaveID;
		}

		void slaveID(uint8_t slave_id) {
			_slaveID = slave_id;
		}

	private:
		//Potrzebuje, zeby tagsDatabase bylo jednoczesnie szybkie do znalezienia przez TagID, oraz przez registerNumber
		// i RegisterType. Najlepiej jeśli jeszcze byłoby posortowane według registerNumber i RegisterType

		std::vector<Tag> _tagsDatabase;
		TagMap _IDtoTagMap;
		std::set<TagID> _excludedTags; //moze powinno to byc excludedregisters?
		std::array<std::set<uint16_t>, 4> _excludedRegisters;
		uint8_t _slaveID;

		bool excludedTagsChanged = false;

		size_t calculateTagHash(const std::span<const TagID> tags) const {
			size_t seed = 0;
			for (const auto &id: tags) {
				// Simple hash combination: seed ^= hash(id) + 0x9e3779b9 + (seed << 6) + (seed >> 2)
				seed ^= std::hash<uint32_t>{}(static_cast<uint32_t>(id)) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
			}
			return seed;
		}

		void sortTags(std::vector<TagID> &tags) {
			std::ranges::sort(tags, [this](const TagID &a_id, const TagID &b_id) {
				const bool a_found = _IDtoTagMap.contains(a_id);
				const bool b_found = _IDtoTagMap.contains(b_id);

				// If neither tag is found, treat them as equivalent
				if (!a_found && !b_found) return false;
				if (!a_found) return false; // Treat 'a' as greater if not found
				if (!b_found) return true; // Treat 'b' as greater if not found

				const Tag &a = getTag(a_id);
				const Tag &b = getTag(b_id);
				// Both tags are found, perform normal comparison
				return (a.register_type < b.register_type) ||
				       (a.register_type == b.register_type && a.register_number < b.register_number);
			});
		}


		bool checkRegistersContinuity(const TagID &first_tag_id, const TagID &end_tag_id) noexcept {
			if (first_tag_id == end_tag_id)
				return true;
			const auto itStart = _IDtoTagMap.find(first_tag_id);
			const auto itEnd = _IDtoTagMap.find(end_tag_id);

			if (itStart == _IDtoTagMap.end() || itEnd == _IDtoTagMap.end()) return false;
			size_t startIndex = itStart->second;
			size_t endIndex = itEnd->second;
			if (startIndex > endIndex) std::swap(startIndex, endIndex);
			const Tag &firstTag = _tagsDatabase[startIndex];
			const Tag &lastTag = _tagsDatabase[endIndex];

			// 3. Different register types can never be "continuous" in one request
			if (firstTag.register_type != lastTag.register_type) return false;
			uint16_t currentReach = firstTag.register_number + firstTag.register_length;

			for (size_t i = startIndex + 1; i <= endIndex; ++i) {
				const Tag &currentTag = _tagsDatabase[i];

				// Gap detected: current tag starts after the previous reach
				if (currentTag.register_number > currentReach) {
					return false;
				}

				// Update reach: some tags might overlap or be sub-ranges of others[cite: 1, 2]
				uint16_t newReach = currentTag.register_number + currentTag.register_length;
				if (newReach > currentReach) {
					currentReach = newReach;
				}
			}

			return true;
		}


		std::vector<Request>* findRequestInCache(const std::span<const TagID> sortedTags) {
			const size_t tagHash = calculateTagHash(sortedTags);
			const auto it = _requestCache.find(tagHash);
			if (it != _requestCache.end()) {
				return &it->second;
			}
			return nullptr;
		}
		void addRequestsToCache(const std::span<const TagID> sortedTags,const std::vector<Request> &requests) {
			const size_t tagHash = calculateTagHash(sortedTags);
			_requestCache.insert({tagHash,requests});
		}
		std::vector<Request> prepareReadRequests(std::initializer_list<const TagID> tags) {
			return prepareReadRequests(std::span(tags));
		}
		std::vector<Request> prepareReadRequests(const std::span<const TagID> tags) {
			std::vector<Request> requests;

			if (tags.empty()) {
				return requests;
			}
			std::vector sortedTags(tags.begin(), tags.end());
			sortTags(sortedTags);
			if (auto* cachedRequests = findRequestInCache(sortedTags)) {
				return *cachedRequests;
			}

			TagID previousTagID{};
			for (const TagID &currentTagID: sortedTags) {
				if (!_IDtoTagMap.contains(currentTagID))
					continue;
				if (_excludedTags.contains(currentTagID))
					continue;

				const Tag &currentTag = getTag(currentTagID);

				if (requests.empty()) {
					requests.push_back({
						.registerType = currentTag.register_type,
						.startAddress = currentTag.register_number,
						.quantity = currentTag.register_length,
						.tagIDs = {currentTagID}
					});
					continue;
				}

				Request &currentRequest = requests.back();

				const bool isSameType = currentRequest.registerType == currentTag.register_type;
				const int distance = currentTag.register_number - currentRequest.startAddress;
				//Check if the distance between the first register in the request and this one is less then the maximum for requests
				const uint16_t currentRegisterEnd = std::max(
					distance + currentTag.register_length, static_cast<int>(currentRequest.quantity));
				const bool registerOffsetLessThanMax = currentRegisterEnd <= TagModbus::MAX_MODBUS_REGISTERS;

				//Check if registers are continuous (if they are not, then the modbus client can reject request)
				const bool registersSpaceContinuous = true || checkRegistersContinuity(previousTagID, currentTagID);

				//Add new position to existing request
				if (isSameType && registerOffsetLessThanMax && registersSpaceContinuous) {
					// uint16_t last_tag_position = current_request.valueCount() - previous_tag.register_length;
					// current_request.setValueCount(last_tag_position  + register_offset + current_tag.register_length);//increase size of the register to pull
					currentRequest.quantity = currentRegisterEnd; //increase size of the register to pull
					currentRequest.tagIDs.push_back(currentTagID);
				} else {
					requests.push_back({
						.registerType = currentTag.register_type,
						.startAddress = currentTag.register_number,
						.quantity = currentTag.register_length,
						.tagIDs = {currentTagID}
					});
					// requests.back().tagIDs.push_back(currentTagID);
				}
			}

			addRequestsToCache(sortedTags,requests);
			return requests;
		}

		bool checkForExcludedRegisters(const RegisterType registerType, uint16_t firstRegisterNumber,
		                               uint16_t lastRegisterNumber) const {
			if (firstRegisterNumber > lastRegisterNumber)
				std::swap(firstRegisterNumber, lastRegisterNumber);
			bool excludedRegistersFound = false;
			for (const auto excludedRegisterNumber: _excludedRegisters[static_cast<int>(registerType)]) {
				if (excludedRegisterNumber >= firstRegisterNumber && excludedRegisterNumber <= lastRegisterNumber) {
					excludedRegistersFound = true;
					break;
				}
			}
			return excludedRegistersFound;
		}

	protected:
		Tag &getTag(const TagID &tagID) {
			return _tagsDatabase[_IDtoTagMap.at(tagID)];
		}
	};
}
